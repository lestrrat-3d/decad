package loftcylinder

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// Affine compares the exact rational maps represented by stored frame and
// transform entries. It is an admission check, not coordinate construction.
// A rounded composition cannot prove two source solids are rigid images of
// the same local source.
type Affine struct {
	Linear [3][3]*big.Rat
	Shift  [3]*big.Rat
}

func AffineOf(t r3.Transform) Affine {
	b := t.Basis()
	columns := [3]r3.Vec{b.EX, b.EY, b.EZ}
	var out Affine
	for j, column := range columns {
		for i, value := range [3]float64{column.X, column.Y, column.Z} {
			out.Linear[i][j] = proofarith.FloatRat(value)
		}
	}
	for i, value := range [3]float64{t.Translation().X, t.Translation().Y, t.Translation().Z} {
		out.Shift[i] = proofarith.FloatRat(value)
	}
	return out
}

func (a Affine) Then(next Affine) Affine {
	var out Affine
	for i := range 3 {
		out.Shift[i] = new(big.Rat).Set(next.Shift[i])
		for k := range 3 {
			out.Shift[i].Add(out.Shift[i], new(big.Rat).Mul(next.Linear[i][k], a.Shift[k]))
		}
		for j := range 3 {
			out.Linear[i][j] = new(big.Rat)
			for k := range 3 {
				out.Linear[i][j].Add(out.Linear[i][j],
					new(big.Rat).Mul(next.Linear[i][k], a.Linear[k][j]))
			}
		}
	}
	return out
}

func (a Affine) Equals(b Affine) bool {
	for i := range 3 {
		if a.Shift[i].Cmp(b.Shift[i]) != 0 {
			return false
		}
		for j := range 3 {
			if a.Linear[i][j].Cmp(b.Linear[i][j]) != 0 {
				return false
			}
		}
	}
	return true
}

// SignedPose admits only a signed permutation with exact unit length,
// determinant +1, and no basis roundoff. Other rotations use the generic
// Boolean until their source-image and output bounds have a separate proof.
func SignedPose(pose r3.Transform) bool {
	if !pose.IsValid() || pose.IsReflection() || !proofbound.FiniteVec(pose.Translation()) {
		return false
	}
	b := pose.Basis()
	used := [3]bool{}
	for _, column := range [3]r3.Vec{b.EX, b.EY, b.EZ} {
		values := [3]float64{column.X, column.Y, column.Z}
		found := -1
		for i, value := range values {
			if value == 0 {
				continue
			}
			if (value != 1 && value != -1) || found >= 0 || used[i] {
				return false
			}
			found = i
		}
		if found < 0 {
			return false
		}
		used[found] = true
	}
	return true
}
