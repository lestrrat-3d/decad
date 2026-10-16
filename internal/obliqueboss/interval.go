// Package obliqueboss builds the held mesh of the oblique cylinder boss-root
// fillet from its source record and proves that mesh's boundary, volume and
// area bounds. Its rational interval-vector arithmetic encloses every sampled
// patch point.
package obliqueboss

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// obliqueIV is one rational interval.
type obliqueIV = proofbound.RatInterval

// obliqueIVPoint is a point or vector whose coordinates are rational intervals.
type obliqueIVPoint struct{ x, y, Z obliqueIV }

// ObliqueRatDot returns the exact rational dot product of two float vectors.
func ObliqueRatDot(a, b r3.Vec) *big.Rat {
	out := new(big.Rat)
	for _, pair := range [3][2]float64{{a.X, b.X}, {a.Y, b.Y}, {a.Z, b.Z}} {
		out.Add(out, new(big.Rat).Mul(proofarith.FloatRat(pair[0]), proofarith.FloatRat(pair[1])))
	}
	return out
}

// ObliqueIVOf returns the point interval holding f exactly.
func ObliqueIVOf(f float64) obliqueIV {
	return proofbound.PointInterval(proofarith.FloatRat(f))
}

// ObliqueZeroPoint returns the all-zero interval point.
func ObliqueZeroPoint() obliqueIVPoint {
	z := ObliqueIVOf(0)
	return obliqueIVPoint{z, z, z}
}

// ObliqueIVVec returns the point interval vector holding v exactly.
func ObliqueIVVec(v r3.Vec) obliqueIVPoint {
	return obliqueIVPoint{ObliqueIVOf(v.X), ObliqueIVOf(v.Y), ObliqueIVOf(v.Z)}
}

// Add returns the componentwise interval sum.
func (p obliqueIVPoint) Add(q obliqueIVPoint) obliqueIVPoint {
	return obliqueIVPoint{
		proofbound.IntervalAdd(p.x, q.x),
		proofbound.IntervalAdd(p.y, q.y),
		proofbound.IntervalAdd(p.Z, q.Z),
	}
}

// Sub returns the componentwise interval difference.
func (p obliqueIVPoint) Sub(q obliqueIVPoint) obliqueIVPoint {
	return obliqueIVPoint{
		proofbound.IntervalSub(p.x, q.x),
		proofbound.IntervalSub(p.y, q.y),
		proofbound.IntervalSub(p.Z, q.Z),
	}
}

// Mul returns p scaled by the interval q.
func (p obliqueIVPoint) Mul(q obliqueIV) obliqueIVPoint {
	return obliqueIVPoint{
		proofbound.IntervalMul(p.x, q),
		proofbound.IntervalMul(p.y, q),
		proofbound.IntervalMul(p.Z, q),
	}
}

// norm encloses the Euclidean length of p.
func (p obliqueIVPoint) norm() (obliqueIV, bool) {
	squares := proofbound.IntervalAdd(proofbound.IntervalSquare(p.x), proofbound.IntervalSquare(p.y))
	squares = proofbound.IntervalAdd(squares, proofbound.IntervalSquare(p.Z))
	return proofbound.SqrtInterval(squares)
}

// Normalized divides p by its enclosed length; false when no positive length is enclosed.
func (p obliqueIVPoint) Normalized() (obliqueIVPoint, bool) {
	norm, ok := p.norm()
	if !ok || norm.Lo.Sign() <= 0 {
		return obliqueIVPoint{}, false
	}
	x, ok := proofbound.IntervalQuo(p.x, norm)
	if !ok {
		return obliqueIVPoint{}, false
	}
	y, ok := proofbound.IntervalQuo(p.y, norm)
	if !ok {
		return obliqueIVPoint{}, false
	}
	z, ok := proofbound.IntervalQuo(p.Z, norm)
	return obliqueIVPoint{x, y, z}, ok
}

// Error bounds the distance between p and the held float vector.
func (p obliqueIVPoint) Error(held r3.Vec) float64 {
	return proofbound.Radius3D(max(
		proofbound.IntervalFloatError(p.x, held.X),
		proofbound.IntervalFloatError(p.y, held.Y),
		proofbound.IntervalFloatError(p.Z, held.Z)))
}
