package facetedrecord

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/loftface"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// UniqueSourcePatches reports whether each named source group has facets
// in the payload and all of them lie in one edge-connected patch, which is the
// condition under which the built body gets exactly one face for the group
// (see uniqueResultSourceFace). It reads the audit's adjacency, so it needs no
// built body.
func UniqueSourcePatches(ctx context.Context, p Payload, adj [][]int, groups ...int) (bool, error) {
	if len(adj) != len(p.Tris) || len(p.Src) != len(p.Tris) {
		return false, nil
	}
	patch, err := facetproof.SourcePatches(proofbound.NewWorkBudget(ctx), len(p.Tris), p.Src, adj)
	if err != nil {
		return false, err
	}
	for _, group := range groups {
		found := -1
		for i, source := range p.Src {
			if source != group {
				continue
			}
			if found >= 0 && found != patch[i] {
				return false, nil
			}
			found = patch[i]
		}
		if found < 0 {
			return false, nil
		}
	}
	return true, nil
}

// The source cap lies outside the toe cone on its whole certified rectangle.
// Linear-fractional coordinate extrema occur at its corners because g > 0.
func CapOutsideToeCone(m loftface.Map, box [4]*big.Rat, cone *PointConeLimit) bool {
	corners := loftface.BoxCorners(box)
	x := loftface.ScalarRange(m, loftface.RatVec{new(big.Rat).SetInt64(1), new(big.Rat), new(big.Rat)}, corners)
	y := loftface.ScalarRange(m, loftface.RatVec{new(big.Rat), new(big.Rat).SetInt64(1), new(big.Rat)}, corners)
	z := loftface.ScalarRange(m, loftface.RatVec{new(big.Rat), new(big.Rat), new(big.Rat).SetInt64(1)}, corners)
	if x.Lo.Cmp(cone.Far) <= 0 {
		return false
	}
	d := loftface.IntervalSub(loftface.Interval{Lo: cone.Apex, Hi: cone.Apex}, x)
	radial := proofarith.AddRat(new(big.Rat), loftface.IntervalSquareMin(y), loftface.IntervalSquareMin(z))
	margin := proofarith.SubRat(new(big.Rat), proofarith.MulRat(new(big.Rat), proofarith.MulRat(new(big.Rat), cone.Slope, cone.Slope), radial), loftface.IntervalSquareMax(d))
	return margin.Sign() > 0
}

// Q is the exact cone pullback after multiplying through the positive map
// denominator. This narrow arm requires Q=A*u²+C*v² with A,C>0 and u
// strictly one-sided throughout the authenticated source rectangle.
func CapOutsideHeelCone(m loftface.Map, box [4]*big.Rat, cone *PointConeLimit) bool {
	if box[1].Sign() >= 0 && box[0].Sign() <= 0 {
		return false
	}
	return loftface.ConeStrictOutsideForm(loftface.ConePullback(m, cone.Apex, cone.Slope))
}
