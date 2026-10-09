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
