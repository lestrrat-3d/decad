package facetedrecord

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/decad/internal/pointcone"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
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

// The larger cone's radial crossing lies farther from the point-loft apex.
// This comparison uses a deliberately wide exact-rational enclosure of the
// source fan, so nearly coincident or crossing cones fall back to mesh Boolean.
func PointConeLimitLater(old, next *PointConeLimit, base Payload) bool {
	return PointConeLimitOrdered(old, next, base)
}

func PointConeLimitEarlier(old, next *PointConeLimit, base Payload) bool {
	return PointConeLimitOrdered(next, old, base)
}

func PointConeLimitOrdered(earlier, later *PointConeLimit, base Payload) bool {
	return pointcone.Ordered(earlier.Apex, earlier.Slope, later.Apex, later.Slope,
		base.Verts, base.MeshBound)
}

func PointConeHasOutsideWitness(base Payload, cone *PointConeLimit) bool {
	move := new(big.Rat).Quo(new(big.Rat).Mul(
		new(big.Rat).Add(big.NewRat(1, 1), cone.Slope),
		proofarith.FloatRat(base.MeshBound)), cone.Apex)
	for _, p := range base.Verts[1:] {
		sample, err := PointConeFactor(p, cone)
		if err != nil {
			continue
		}
		margin := new(big.Rat).Sub(proofarith.FloatRat(sample.Q), big.NewRat(1, 1))
		margin.Sub(margin, proofarith.FloatRat(sample.QError))
		if margin.Cmp(move) > 0 {
			return true
		}
	}
	return false
}

func PointConeSubdivide(ctx context.Context, base Payload, rounds int) (
	[]r3.Vec, [][3]int, []pointcone.Side, float64, error) {
	if base.PointSection == nil {
		return nil, nil, nil, 0, fmt.Errorf("%w: the point-cone source has no fan", decaderr.ErrUnsupported)
	}
	return pointcone.Subdivide(ctx, base.Verts, base.Tris, base.Src, rounds)
}
