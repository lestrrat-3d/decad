package facetedrecord

import (
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/decad/internal/pointcone"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// PointConeFacetGroup retains a cone's source face in a faceted result.
func PointConeFacetGroup(cone *PointConeLimit, flip bool) Group {
	denoted := cone.Denoted
	if flip {
		// A straight revolve wall's denotation stores its oriented meridian
		// runs. Its Allow method reads that orientation directly.
		denotedCopy := *denoted
		denotedCopy.Runs = append([][2]proofbound.RatInterval(nil), denoted.Runs...)
		for i := range denotedCopy.Runs {
			denotedCopy.Runs[i][0] = proofbound.IntervalNeg(denotedCopy.Runs[i][0])
			denotedCopy.Runs[i][1] = proofbound.IntervalNeg(denotedCopy.Runs[i][1])
		}
		denoted = &denotedCopy
	}
	return Group{Origins: cone.Origins, Surface: cone.Surface,
		Denoted: denoted, Reversed: cone.Reversed != flip}
}

// PointConeFactor dispatches a held far-cap point to the Dyadic crossing for a
// positive apex and slope, and to the big.Rat form otherwise.
func PointConeFactor(p r3.Vec, cone *PointConeLimit) (pointcone.Sample, error) {
	if cone.Apex.Sign() <= 0 || cone.Slope.Sign() <= 0 {
		return pointcone.FactorRat(p, cone.Apex, cone.Slope)
	}
	return pointcone.FactorDyadic(p, cone.Constants())
}

// PointConeFactorParallelMin is the point count from which PointConeFactors
// callers ask for more than one worker.
const PointConeFactorParallelMin = 1024

// PointConeFactors is PointConeFactor over every point: samples[i] and errs[i]
// are point i's. With workers below 2, or a non-positive apex or slope (the
// big.Rat form), it runs in point order on the calling goroutine and stops
// after the first error, leaving later slots zero. Otherwise the cone
// constants are resolved first and the points are spread over workers; the
// lowest non-nil errs index is then the serial first error, and slots past it
// may or may not be filled.
func PointConeFactors(points []r3.Vec, cone *PointConeLimit, workers int) ([]pointcone.Sample, []error) {
	samples := make([]pointcone.Sample, len(points))
	errs := make([]error, len(points))
	if cone.Apex.Sign() <= 0 || cone.Slope.Sign() <= 0 {
		workers = 1
	}
	if workers < 2 {
		for i, p := range points {
			samples[i], errs[i] = PointConeFactor(p, cone)
			if errs[i] != nil {
				break
			}
		}
		return samples, errs
	}
	consts := cone.Constants()
	// The per-index error lives in errs, so Each's own result is unused.
	_, _ = orderedwork.Each(len(points), workers, 256, nil, func(i int) error {
		samples[i], errs[i] = pointcone.FactorDyadic(points[i], consts)
		return errs[i]
	})
	return samples, errs
}
