package twistsweep

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// straightTwistCentroid places the centroid of a straight-path twist of
// height h and denoted angle angleIv. A section whose centroid c is on the
// axis keeps (0, 0, h/2). Otherwise the section turns by theta*z/h about the
// axis, so the centroid's plane part is
// (1/theta) * integral over [0, theta] of R(phi)*c, that is
// ((sin t*cu - (1-cos t)*cv), ((1-cos t)*cu + sin t*cv)) / t.
// Exact rational sine/cosine intervals enclose it; the returned bound adds
// the enclosure width, the held rounding and the placement rounding.
func straightTwistCentroid(poly TwistPolygon, height float64, angleIv proofbound.RatInterval,
	xform r3.Transform) (r3.Vec, float64, error) {
	if !poly.OffAxis() {
		center := xform.Apply(r3.NewVec(0, 0, height/2))
		bound := 0.0
		if xform != r3.Identity() {
			tr := xform.Translation()
			bound = proofbound.RigidRoundAllow(height/2, max(math.Abs(tr.X), math.Abs(tr.Y), math.Abs(tr.Z)))
		}
		return center, bound, nil
	}
	sin, cos, ok := proofbound.RadSinCosSpan(angleIv)
	if !ok {
		return r3.Vec{}, 0, fmt.Errorf(`%w: twisted centroid has no trigonometric enclosure`, decaderr.ErrUnsupported)
	}
	oneMinusCos := proofbound.IntervalSub(proofbound.PointInterval(big.NewRat(1, 1)), cos)
	numX := proofbound.IntervalSub(proofbound.IntervalScale(sin, poly.CentroidU),
		proofbound.IntervalScale(oneMinusCos, poly.CentroidV))
	numY := proofbound.IntervalAdd(proofbound.IntervalScale(oneMinusCos, poly.CentroidU),
		proofbound.IntervalScale(sin, poly.CentroidV))
	x, okX := proofbound.IntervalQuo(numX, angleIv)
	y, okY := proofbound.IntervalQuo(numY, angleIv)
	if !okX || !okY {
		return r3.Vec{}, 0, fmt.Errorf(`%w: twisted centroid has no bounded turn`, decaderr.ErrUnsupported)
	}
	mid := func(a proofbound.RatInterval) float64 {
		m, _ := new(big.Rat).Quo(new(big.Rat).Add(a.Lo, a.Hi), big.NewRat(2, 1)).Float64()
		return m
	}
	local := r3.NewVec(mid(x), mid(y), height/2)
	planar := proofbound.AbsSumUpper(proofbound.IntervalFloatError(x, local.X),
		proofbound.IntervalFloatError(y, local.Y))
	tr := xform.Translation()
	round := 0.0
	if xform != r3.Identity() {
		round = proofbound.RigidRoundAllow(max(math.Abs(local.X), math.Abs(local.Y), height/2),
			max(math.Abs(tr.X), math.Abs(tr.Y), math.Abs(tr.Z)))
	}
	bound := proofbound.AbsSumUpper(planar, round)
	if proofbound.IsNonFinite(bound) {
		return r3.Vec{}, 0, fmt.Errorf(`%w: twisted centroid has no finite bound`, decaderr.ErrUnsupported)
	}
	return xform.Apply(local), bound, nil
}
