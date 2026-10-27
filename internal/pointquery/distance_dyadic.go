package pointquery

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
)

// dyDistance keeps a nonnegative squared distance as n/d, d > 0. Every
// coordinate is dyadic, so the distance formula needs no division until the
// final result is returned. Candidate comparisons are exact cross products.
type dyDistance struct {
	n, d proof.Dyadic
}

func (a dyDistance) cmp(b dyDistance) int {
	return proof.DyCmp(proof.DyMul(a.n, b.d), proof.DyMul(b.n, a.d))
}

func (a dyDistance) rat() *big.Rat {
	return new(big.Rat).Quo(a.n.Rat(), a.d.Rat())
}

func segmentDyDistanceSquared(p, a, b proof.DyV3) dyDistance {
	axis, fromStart := proof.DvSub(b, a), proof.DvSub(p, a)
	lengthSquared := proof.DvDot(axis, axis)
	startSquared := proof.DvDot(fromStart, fromStart)
	if lengthSquared.Sign() == 0 {
		return dyDistance{startSquared, proof.DyInt(1)}
	}
	along := proof.DvDot(fromStart, axis)
	if along.Sign() <= 0 {
		return dyDistance{startSquared, proof.DyInt(1)}
	}
	if proof.DyCmp(along, lengthSquared) >= 0 {
		fromEnd := proof.DvSub(p, b)
		return dyDistance{proof.DvDot(fromEnd, fromEnd), proof.DyInt(1)}
	}
	// |p-a|² - along²/lengthSquared, with the division deferred. The
	// numerator is nonnegative by the exact Cauchy-Schwarz inequality.
	return dyDistance{
		proof.DySubScalar(proof.DyMul(startSquared, lengthSquared), proof.DyMul(along, along)),
		lengthSquared,
	}
}

func triangleDyDistanceSquared(p, a, b, c proof.DyV3) (dyDistance, error) {
	normal := proof.DvCross(proof.DvSub(b, a), proof.DvSub(c, a))
	normalSquared := proof.DvDot(normal, normal)
	if normalSquared.Sign() == 0 {
		return dyDistance{}, fmt.Errorf(`%w: point query mesh contains a degenerate triangle`, decaderr.ErrUnsupported)
	}
	triangle := [3]proof.DyV3{a, b, c}
	projectionInside := true
	for i := range triangle {
		start, end := triangle[i], triangle[(i+1)%3]
		side := proof.DvDot(proof.DvCross(proof.DvSub(end, start), proof.DvSub(p, start)), normal)
		if side.Sign() < 0 {
			projectionInside = false
			break
		}
	}
	if projectionInside {
		offset := proof.DvDot(proof.DvSub(p, a), normal)
		return dyDistance{proof.DyMul(offset, offset), normalSquared}, nil
	}
	minimum := segmentDyDistanceSquared(p, a, b)
	for i := 1; i < len(triangle); i++ {
		distance := segmentDyDistanceSquared(p, triangle[i], triangle[(i+1)%3])
		if distance.cmp(minimum) < 0 {
			minimum = distance
		}
	}
	return minimum, nil
}

func boxDyDistanceSquared(p, a, b, c proof.DyV3) proof.Dyadic {
	result := proof.DyZero()
	for i := range p {
		lo, hi := proof.DyMin(a[i], proof.DyMin(b[i], c[i])), proof.DyMax(a[i], proof.DyMax(b[i], c[i]))
		var gap proof.Dyadic
		switch {
		case proof.DyCmp(p[i], lo) < 0:
			gap = proof.DySubScalar(lo, p[i])
		case proof.DyCmp(p[i], hi) > 0:
			gap = proof.DySubScalar(p[i], hi)
		default:
			continue
		}
		result = proof.DyAdd(result, proof.DyMul(gap, gap))
	}
	return result
}
