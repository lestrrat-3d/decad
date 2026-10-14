package extent

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

const stopTol = 1e-9

// StopLevelRound bounds the rounding in a face-relative stop level.
func StopLevelRound(faceOrigin, planeOrigin, n r3.Vec, travel, offset, held float64) float64 {
	face := [3]float64{faceOrigin.X, faceOrigin.Y, faceOrigin.Z}
	plane := [3]float64{planeOrigin.X, planeOrigin.Y, planeOrigin.Z}
	normal := [3]float64{n.X, n.Y, n.Z}
	terms := make([]*big.Rat, 0, 4)
	for i := range face {
		f, p, nn := proofarith.FloatRat(face[i]), proofarith.FloatRat(plane[i]), proofarith.FloatRat(normal[i])
		if f == nil || p == nil || nn == nil {
			return math.Inf(1)
		}
		terms = append(terms, proofbound.RatMul(new(big.Rat).Sub(f, p), nn))
	}
	t, o := proofarith.FloatRat(travel), proofarith.FloatRat(offset)
	if t == nil || o == nil {
		return math.Inf(1)
	}
	terms = append(terms, proofbound.RatMul(t, o))
	return proofarith.RationalFloatError(proofbound.RatAdd(terms...), held)
}

// ExactlyParallel reports whether a and b are parallel exactly, in either
// sense: their cross product, computed in rationals from the held float
// components, is zero. A zero or non-finite vector is never parallel.
func ExactlyParallel(a, b r3.Vec) bool {
	av, bv := [3]float64{a.X, a.Y, a.Z}, [3]float64{b.X, b.Y, b.Z}
	var ar, br [3]*big.Rat
	for i := range av {
		ar[i], br[i] = proofarith.FloatRat(av[i]), proofarith.FloatRat(bv[i])
		if ar[i] == nil || br[i] == nil {
			return false
		}
	}
	if (ar[0].Sign() == 0 && ar[1].Sign() == 0 && ar[2].Sign() == 0) ||
		(br[0].Sign() == 0 && br[1].Sign() == 0 && br[2].Sign() == 0) {
		return false
	}
	for i := range ar {
		j, k := (i+1)%3, (i+2)%3
		if new(big.Rat).Sub(proofbound.RatMul(ar[j], br[k]), proofbound.RatMul(ar[k], br[j])).Sign() != 0 {
			return false
		}
	}
	return true
}

// throughStopRound bounds the rounding in a through-all stop level.
func throughStopRound(origin, dir r3.Vec, hi, travel, held float64) float64 {
	o := [3]float64{origin.X, origin.Y, origin.Z}
	g := [3]float64{dir.X, dir.Y, dir.Z}
	base := new(big.Rat)
	for i := range o {
		oi, gi := proofarith.FloatRat(o[i]), proofarith.FloatRat(g[i])
		if oi == nil || gi == nil {
			return math.Inf(1)
		}
		base.Add(base, proofbound.RatMul(oi, gi))
	}
	h, t := proofarith.FloatRat(hi), proofarith.FloatRat(travel)
	if h == nil || t == nil {
		return math.Inf(1)
	}
	return proofarith.RationalFloatError(new(big.Rat).Mul(t, new(big.Rat).Sub(h, base)), held)
}

// RelativeStopTolerance scales the stop contact tolerance to the input.
func RelativeStopTolerance(scale float64) float64 {
	return stopTol * math.Max(1, math.Abs(scale))
}
