package box

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// ProvesDeparture reports whether an initially touching pair of exact source
// boxes separates strictly along an opposed support axis under affine motion.
func ProvesDeparture(a, b AxisBox, da, db [3]proofarith.Dyadic) bool {
	for axis := range 3 {
		if proofarith.DyCmp(a.Hi[axis], b.Lo[axis]) == 0 {
			if proofarith.DySubScalar(db[axis], da[axis]).Sign() > 0 {
				return true
			}
		}
		if proofarith.DyCmp(b.Hi[axis], a.Lo[axis]) == 0 {
			if proofarith.DySubScalar(da[axis], db[axis]).Sign() > 0 {
				return true
			}
		}
	}
	return false
}

// ContactSpan intersects six exact linear support inequalities. It returns
// the first and last fractions where the closed source boxes can meet.
func ContactSpan(a, b AxisBox, da, db [3]proofarith.Dyadic) (*big.Rat, *big.Rat, bool) {
	entry, exit := new(big.Rat), big.NewRat(1, 1)
	for axis := range 3 {
		constraints := [2][2]proofarith.Dyadic{
			{proofarith.DySubScalar(a.Hi[axis], b.Lo[axis]), proofarith.DySubScalar(da[axis], db[axis])},
			{proofarith.DySubScalar(b.Hi[axis], a.Lo[axis]), proofarith.DySubScalar(db[axis], da[axis])},
		}
		for _, c := range constraints {
			start, slope := c[0], c[1]
			if slope.IsZero() {
				if start.Sign() < 0 {
					return entry, exit, false
				}
				continue
			}
			root := new(big.Rat).Quo(proofarith.DyNeg(start).Rat(), slope.Rat())
			if slope.Sign() > 0 && root.Cmp(entry) > 0 {
				entry = root
			}
			if slope.Sign() < 0 && root.Cmp(exit) < 0 {
				exit = root
			}
		}
	}
	return entry, exit, entry.Cmp(exit) <= 0
}
