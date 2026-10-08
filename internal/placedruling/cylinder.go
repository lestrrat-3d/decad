// Package placedruling computes exact support geometry for a placed cylinder
// against a planar body.
package placedruling

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// Cylinder holds the exact geometric data of a source cylinder at a query pose.
type Cylinder struct {
	Axis         int
	Radius, Gram proofarith.Dyadic
	BoxLo, BoxHi [3]proofarith.Dyadic
	Centers      [2]proofarith.DyV3
	Columns      [3]proofarith.DyV3
}

// SectionDrift is r·gram + r·α², the bound on how far the staged section's
// least height along a unit normal n̂ departs from its center's height less
// r, with α = n̂·Bâ the normal component of the staged axis column: that least
// height is r·|P·Bᵀn̂| below the center, P removing the identity axis, and
// |P·Bᵀn̂|² = |Bᵀn̂|² − α² lies in [1 − gram − α², 1 + gram], so
// |r − r·|P·Bᵀn̂|| <= r·(gram + α²).
func (c Cylinder) SectionDrift(alpha proofarith.Dyadic) proofarith.Dyadic {
	return proofarith.DyMul(c.Radius, proofarith.DyAdd(c.Gram, proofarith.DyMul(alpha, alpha)))
}

// RimDrift is r·(3·gram + (3/2)·|α|): how far the true lowest point of a
// staged end disk lies from its center less r·n̂, while gram <= 1/16 and
// |α| <= 1/4. With A the staged basis, y = P·Aᵀn̂ and AAᵀ = I + E, ‖E‖ <= gram,
// the lowest point is c − r·Ay/|y| and Ay = n̂ + E·n̂ − α·Aâ, so
// |Ay − |y|·n̂| <= (gram + α²) + gram + |α|·√(1 + gram), and dividing by
// |y| >= √(1 − gram − α²) >= √(7/8) leaves at most 3·gram + (3/2)·|α|.
func (c Cylinder) RimDrift(alpha *big.Rat) *big.Rat {
	return proofbound.RatMul(c.Radius.Rat(), proofbound.RatAdd(proofbound.RatMul(big.NewRat(3, 1), c.Gram.Rat()),
		proofbound.RatMul(big.NewRat(3, 2), new(big.Rat).Abs(alpha))))
}

// StagedCorners are the eight corners of the cylinder's identity
// disk-by-interval box under the pose, whose hull holds the staged cylinder.
// The box's center maps to the midpoint of the staged end centers and each
// half extent along a basis column, so every corner is exact.
func (c Cylinder) StagedCorners() [8]proofarith.DyV3 {
	var mid proofarith.DyV3
	var extent [3]proofarith.Dyadic
	for k := range 3 {
		mid[k] = proofarith.DyShift(proofarith.DyAdd(c.Centers[0][k], c.Centers[1][k]), -1)
		extent[k] = proofarith.DyShift(proofarith.DySubScalar(c.BoxHi[k], c.BoxLo[k]), -1)
	}
	var out [8]proofarith.DyV3
	for i := range out {
		corner := mid
		for k := range 3 {
			step := extent[k]
			if i&(1<<k) == 0 {
				step = proofarith.DyNeg(step)
			}
			corner = proofarith.DvAdd(corner, scaleVec(c.Columns[k], step))
		}
		out[i] = corner
	}
	return out
}

func scaleVec(v proofarith.DyV3, s proofarith.Dyadic) proofarith.DyV3 {
	return proofarith.DyV3{proofarith.DyMul(v[0], s), proofarith.DyMul(v[1], s), proofarith.DyMul(v[2], s)}
}
