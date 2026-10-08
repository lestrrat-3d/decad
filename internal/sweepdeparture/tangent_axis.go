package sweepdeparture

import (
	"math/big"

	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// Path contains the exact prepared readings used by source-box departure proofs.
type Path struct {
	Box      pairbox.OrientedBox
	Delta    [3]proofarith.Dyadic
	Axis     [3]*big.Rat
	Center   [3]*big.Rat
	Velocity [3]*big.Rat
	Duration *big.Rat
	Drift    bool
	Screw    bool
}

// ContactNormal records one published manifold normal and whether its bound
// and angle are both exactly zero.
type ContactNormal struct {
	Value r3.Vec
	Exact bool
}

// TangentAxisSpin proves a whole-body gap for a source box spinning about Y
// above a stationary horizontal face. It preserves the original 60-step
// dyadic horizon search and exact-rational derivative and curvature bounds.
func TangentAxisSpin(paths [2]Path, normals []ContactNormal) (*big.Rat, bool) {
	if len(normals) != 4 {
		return nil, false
	}
	stationary, spinning := -1, -1
	for i, path := range paths {
		if !path.Drift && path.Delta == [3]proofarith.Dyadic{} {
			stationary = i
		}
		if path.Drift && !path.Screw &&
			path.Axis[0].Sign() == 0 && path.Axis[1].Sign() != 0 && path.Axis[2].Sign() == 0 {
			spinning = i
		}
	}
	if stationary < 0 || spinning < 0 || stationary == spinning {
		return nil, false
	}
	static, moving := &paths[stationary].Box, &paths[spinning].Box
	staticLow, staticHigh := static.Corner[0][2], static.Corner[0][2]
	movingLow, movingHigh := moving.Corner[0][2], moving.Corner[0][2]
	for i := 1; i < len(static.Corner); i++ {
		staticLow = dyMin(staticLow, static.Corner[i][2])
		staticHigh = dyMax(staticHigh, static.Corner[i][2])
		movingLow = dyMin(movingLow, moving.Corner[i][2])
		movingHigh = dyMax(movingHigh, moving.Corner[i][2])
	}
	sign := int64(0)
	if proofarith.DyCmp(staticHigh, movingLow) == 0 {
		sign = 1
	} else if proofarith.DyCmp(movingHigh, staticLow) == 0 {
		sign = -1
	}
	if sign == 0 {
		return nil, false
	}
	wantNormal := float64(sign)
	if spinning == 0 {
		wantNormal = -wantNormal
	}
	for _, point := range normals {
		if point.Value != (r3.Vec{Z: wantNormal}) || !point.Exact {
			return nil, false
		}
	}
	path := paths[spinning]
	omega := path.Axis[1]
	omegaSquared := new(big.Rat).Mul(omega, omega)
	minimum, curvature := new(big.Rat), new(big.Rat)
	for i := range moving.Corner {
		corner := &moving.Corner[i]
		dx := new(big.Rat).Sub(corner[0].Rat(), path.Center[0])
		dz := new(big.Rat).Sub(corner[2].Rat(), path.Center[2])
		derivative := new(big.Rat).Sub(path.Velocity[2], new(big.Rat).Mul(omega, dx))
		derivative.Mul(derivative, big.NewRat(sign, 1))
		if i == 0 || derivative.Cmp(minimum) < 0 {
			minimum = derivative
		}
		cornerCurvature := new(big.Rat).Mul(omegaSquared,
			new(big.Rat).Add(new(big.Rat).Abs(dx), new(big.Rat).Abs(dz)))
		if cornerCurvature.Cmp(curvature) > 0 {
			curvature = cornerCurvature
		}
	}
	if minimum.Sign() <= 0 {
		return nil, false
	}
	fraction := big.NewRat(1, 1)
	for range 60 {
		until := new(big.Rat).Mul(fraction, paths[0].Duration)
		if new(big.Rat).Mul(curvature, until).Cmp(minimum) < 0 {
			return fraction, true
		}
		fraction.Quo(fraction, big.NewRat(2, 1))
	}
	return nil, false
}

func dyMax(a, b proofarith.Dyadic) proofarith.Dyadic {
	if proofarith.DyCmp(a, b) >= 0 {
		return a
	}
	return b
}

func dyMin(a, b proofarith.Dyadic) proofarith.Dyadic {
	if proofarith.DyCmp(a, b) <= 0 {
		return a
	}
	return b
}
