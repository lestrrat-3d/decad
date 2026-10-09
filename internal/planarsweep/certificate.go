package planarsweep

import (
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/pair/planar"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sweepmemo"
)

// DepartureHeight bounds the distance from the guest's vertices to the
// owner's support plane at elapsed time t. A nonpositive height is returned
// before division, matching the departure certificate's refusal path.
func DepartureHeight(heights, rates, curvature []*big.Rat, t, nHigh *big.Rat) *big.Rat {
	var least *big.Rat
	for i, height := range heights {
		value := proofbound.RatAdd(height, proofbound.RatMul(rates[i], t))
		value.Sub(value, proofbound.RatMul(curvature[i], t, t))
		if least == nil || value.Cmp(least) < 0 {
			least = value
		}
	}
	if least.Sign() <= 0 {
		return least
	}
	return least.Quo(least, nHigh)
}

// FaceContains proves that each held contact vertex's projected path box,
// widened by depth, remains inside the support face through fraction f.
func FaceContains(face *planar.SupportFace, spans sweepmemo.CornerSpans, delta [3]proofarith.Dyadic,
	contact, lifted []int, f, depth *big.Rat, poll func() error) (bool, error) {
	i, j := (face.Drop+1)%3, (face.Drop+2)%3
	for _, index := range slices.Concat(contact, lifted) {
		if err := poll(); err != nil {
			return false, err
		}
		var lo, hi [2]*big.Rat
		for slot, axis := range [2]int{i, j} {
			shift := new(big.Rat).Mul(delta[axis].Rat(), f)
			shiftLo, shiftHi := proofbound.RatMin(shift, new(big.Rat)), proofbound.RatMax(shift, new(big.Rat))
			span := spans.Span(index, axis)
			lo[slot] = proofbound.RatAdd(span.Lo, new(big.Rat).Neg(shiftHi), new(big.Rat).Neg(depth))
			hi[slot] = proofbound.RatAdd(span.Hi, new(big.Rat).Neg(shiftLo), depth)
		}
		if !face.HoldsBox(lo, hi) {
			return false, nil
		}
	}
	return true, nil
}

// ReplayHeights checks the rounded guest vertices against the held band on
// the rounded owner's plane. Both pose deviations are charged by the caller.
func ReplayHeights(verts []proofarith.DyV3, origin, normal proofarith.DyV3,
	heldDepth, deviation, nHigh *big.Rat) bool {
	floor := new(big.Rat).Neg(proofbound.RatMul(new(big.Rat).Add(heldDepth, deviation), nHigh))
	for _, v := range verts {
		if proofarith.DvDot(normal, proofarith.DvSub(v, origin)).Rat().Cmp(floor) < 0 {
			return false
		}
	}
	return true
}
