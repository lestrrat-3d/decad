// Package sphere proves contact between complete source balls.
package sphere

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/pair"
	"github.com/lestrrat-3d/decad/internal/pair/box"
	proof "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sweeppath"
	"github.com/lestrrat-3d/r3"
)

// Ball is a complete occupied sphere admitted by its source record.
type Ball struct {
	Center proof.DyV3
	Radius proof.Dyadic
}

// Witness contains bounded points on both original sphere faces.
type Witness struct {
	OnA, OnB   box.PointReading
	Normal     box.NormalReading
	Separation pair.ScalarReading
}

// Result carries an exact relation and an optional bounded response witness.
type Result struct {
	Relation pair.Relation
	Reason   pair.Reason
	Gap      *pair.ScalarReading
	Witness  *Witness
}

// Classify compares complete occupied balls. A response witness needs a
// nonzero center line and two crossing sphere faces.
func Classify(a, b Ball, pointResolutionMM, normalResolutionRad float64) Result {
	var delta proof.DyV3
	distance2 := proof.DyZero()
	axis, nonzero := 0, 0
	for i := range 3 {
		delta[i] = proof.DySubScalar(b.Center[i], a.Center[i])
		distance2 = proof.DyAdd(distance2, proof.DyMul(delta[i], delta[i]))
		if delta[i].Sign() != 0 {
			axis, nonzero = i, nonzero+1
		}
	}
	radius := proof.DyAdd(a.Radius, b.Radius)
	radius2 := proof.DyMul(radius, radius)
	result := Result{}
	switch proof.DyCmp(distance2, radius2) {
	case 1:
		gap, ok := separatedGap(distance2, radius)
		if !ok {
			result.Reason = pair.NoGapProof
			return result
		}
		result.Relation, result.Gap = pair.Separated, &gap
		return result
	case 0:
		zero := pair.ScalarReading{}
		result.Relation, result.Gap = pair.Touching, &zero
	default:
		result.Relation = pair.Overlapping
	}
	if nonzero == 0 {
		result.Reason = pair.AmbiguousFeature
		return result
	}
	radiusDifference := proof.DyAbs(proof.DySubScalar(a.Radius, b.Radius))
	if proof.DyCmp(distance2, proof.DyMul(radiusDifference, radiusDifference)) <= 0 {
		result.Reason = pair.AmbiguousFeature
		return result
	}
	if nonzero != 1 {
		result.Witness, result.Reason = obliqueWitness(a, b, delta, distance2, radius,
			pointResolutionMM, normalResolutionRad)
		return result
	}
	sign := 1.0
	if delta[axis].Sign() < 0 {
		sign = -1
	}
	normal := r3.Vec{}
	switch axis {
	case 0:
		normal.X = sign
	case 1:
		normal.Y = sign
	case 2:
		normal.Z = sign
	}
	onAExact, onBExact := a.Center, b.Center
	onAExact[axis] = proof.DyAdd(a.Center[axis], proof.DyMul(proof.MustDyOf(sign), a.Radius))
	onBExact[axis] = proof.DySubScalar(b.Center[axis], proof.DyMul(proof.MustDyOf(sign), b.Radius))
	onA, okA := box.ReadPointAt(&onAExact)
	onB, okB := box.ReadPointAt(&onBExact)
	if !okA || !okB || onA.BoundMM > pointResolutionMM || onB.BoundMM > pointResolutionMM {
		result.Reason = pair.PointTooCoarse
		return result
	}
	distance := proof.DyAbs(delta[axis])
	sep, ok := box.SignedReading(proof.DySubScalar(distance, radius))
	if !ok {
		result.Reason = pair.PointTooCoarse
		return result
	}
	result.Witness = &Witness{OnA: onA, OnB: onB,
		Normal: box.NormalReading{Value: normal}, Separation: sep}
	return result
}

func separatedGap(distance2, radius proof.Dyadic) (pair.ScalarReading, bool) {
	lower := new(big.Rat).Sub(proof.FloatRat(proof.DySqrtDown(distance2)), radius.Rat())
	upper := new(big.Rat).Sub(proof.FloatRat(proof.DySqrtUp(distance2)), radius.Rat())
	lo, hi := proofbound.RatFloatDown(lower), proofbound.RatFloatUp(upper)
	if !finite(lo, hi) || lo <= 0 || hi < lo {
		return pair.ScalarReading{}, false
	}
	value := lo + (hi-lo)/2
	left := new(big.Rat).Sub(proof.FloatRat(value), proof.FloatRat(lo))
	right := new(big.Rat).Sub(proof.FloatRat(hi), proof.FloatRat(value))
	if right.Cmp(left) > 0 {
		left = right
	}
	bound := proofbound.RatFloatUp(left)
	if !finite(value, bound) ||
		new(big.Rat).Sub(proof.FloatRat(value), proof.FloatRat(bound)).Sign() <= 0 {
		return pair.ScalarReading{}, false
	}
	return pair.ScalarReading{ValueMM: value, BoundMM: bound}, true
}

// obliqueWitness retains the exact center line until final conversion. Each
// point bound includes normal conversion error multiplied by its source radius.
func obliqueWitness(a, b Ball, delta proof.DyV3, distance2, radius proof.Dyadic,
	pointResolutionMM, normalResolutionRad float64) (*Witness, pair.Reason) {
	normal, ok := box.DyadicNormalReading(delta)
	if !ok || normal.Angle > normalResolutionRad {
		return nil, pair.NoNormalProof
	}
	components := [3]float64{normal.Value.X, normal.Value.Y, normal.Value.Z}
	var pointA, pointB [3]*big.Rat
	for i, component := range components {
		offsetA := new(big.Rat).Mul(a.Radius.Rat(), proof.FloatRat(component))
		offsetB := new(big.Rat).Mul(b.Radius.Rat(), proof.FloatRat(component))
		pointA[i] = new(big.Rat).Add(a.Center[i].Rat(), offsetA)
		pointB[i] = new(big.Rat).Sub(b.Center[i].Rat(), offsetB)
	}
	onA, okA := box.RationalPointReading(pointA)
	onB, okB := box.RationalPointReading(pointB)
	if !okA || !okB {
		return nil, pair.PointTooCoarse
	}
	for _, witness := range []struct {
		point  *box.PointReading
		radius proof.Dyadic
	}{{&onA, a.Radius}, {&onB, b.Radius}} {
		radiusFloat := proofbound.RatFloatUp(witness.radius.Rat())
		bound := proofbound.ProvenUpRound(witness.point.BoundMM +
			proofbound.ProvenUpRound(radiusFloat*normal.Bound))
		if !finite(bound) || bound > pointResolutionMM {
			return nil, pair.PointTooCoarse
		}
		witness.point.BoundMM = bound
	}
	low := new(big.Rat).Sub(proof.FloatRat(proof.DySqrtDown(distance2)), radius.Rat())
	high := new(big.Rat).Sub(proof.FloatRat(proof.DySqrtUp(distance2)), radius.Rat())
	value := sweeppath.RatFloatNearest(new(big.Rat).Quo(new(big.Rat).Add(low, high), big.NewRat(2, 1)))
	if !finite(value) {
		return nil, pair.PointTooCoarse
	}
	left := new(big.Rat).Sub(low, proof.FloatRat(value))
	right := new(big.Rat).Sub(high, proof.FloatRat(value))
	boundExact := proofbound.RatMax(left.Abs(left), right.Abs(right))
	boundExact.Add(boundExact, proof.FloatRat(onA.BoundMM))
	boundExact.Add(boundExact, proof.FloatRat(onB.BoundMM))
	bound := proofbound.RatFloatUp(boundExact)
	if !finite(bound) {
		return nil, pair.PointTooCoarse
	}
	return &Witness{OnA: onA, OnB: onB, Normal: normal,
		Separation: pair.ScalarReading{ValueMM: value, BoundMM: bound}}, pair.NoReason
}

func finite(values ...float64) bool {
	for _, value := range values {
		if math.IsNaN(value) || math.IsInf(value, 0) {
			return false
		}
	}
	return true
}
