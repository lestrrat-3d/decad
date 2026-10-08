package sweepdeparture

import (
	"math/big"

	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// HorizontalSpin proves a whole-body Z gap when a source box spins around Z
// while moving outward from an exactly touching stationary box.
func HorizontalSpin(paths [2]Path, normal r3.Vec,
	sourceBox func(int) (pairbox.OrientedBox, bool)) (*big.Rat, bool) {
	stationary, spinning := -1, -1
	for i, path := range paths {
		if !path.Drift && path.Delta == [3]proofarith.Dyadic{} {
			stationary = i
		}
		if path.Drift && path.Axis[0].Sign() == 0 &&
			path.Axis[1].Sign() == 0 && path.Axis[2].Sign() != 0 &&
			path.Velocity[0].Sign() == 0 && path.Velocity[1].Sign() == 0 {
			spinning = i
		}
	}
	if stationary < 0 || spinning < 0 || stationary == spinning {
		return nil, false
	}
	if normal != (r3.Vec{Z: 1}) && normal != (r3.Vec{Z: -1}) {
		return nil, false
	}
	a, okA := sourceBox(0)
	b, okB := sourceBox(1)
	if !okA || !okB {
		return nil, false
	}
	z := proofarith.DyV3{proofarith.DyZero(), proofarith.DyZero(), proofarith.MustDyOf(1)}
	alo, ahi := pairbox.OrientedProjection(a, z)
	blo, bhi := pairbox.OrientedProjection(b, z)
	gap := proofarith.DySubScalar(blo, ahi)
	if normal.Z < 0 {
		gap = proofarith.DySubScalar(alo, bhi)
	}
	if gap.Sign() != 0 {
		return nil, false
	}
	speed := new(big.Rat).Set(paths[spinning].Velocity[2])
	if spinning == 0 {
		speed.Neg(speed)
	}
	if normal.Z < 0 {
		speed.Neg(speed)
	}
	if speed.Sign() <= 0 {
		return nil, false
	}
	return big.NewRat(1, 2), true
}
