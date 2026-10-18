package spiralsector

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// ShortHeldBound compares an authenticated finite cap intersection
// with one complete held switch chain. The caller authenticates the source
// family, the chain's face roles and topology, and minBound against every
// public curve bound on the chain.
//
// Each source cell contains a nonempty part of the true intersection. For
// fitted flanks, the cell at the finite cap start may extend across its
// isolated clip, which only enlarges its box. Every source cell box must be
// near a held edge box, and every held edge box must be near a source cell
// box. The maximum possible distance between either pair of boxes proves
// both directions, including their endpoints, without assuming that X or
// angle is ordered on the source curve.
func ShortHeldBound(ctx context.Context, proof *facetedrecord.SpiralFirstJoinSource,
	family ShortCapFamily, held []r3.Vec, minBound float64) (float64, bool, error) {
	if ctx == nil || proof == nil || proof.Tooth == nil || len(proof.Tooth.Maps) != 9 ||
		len(family.Pieces) != 1 || len(held) < 2 || len(held) > 1025 ||
		proofbound.IsNonFinite(minBound) || minBound <= 0 ||
		family.Band != 1 && family.Band != 7 ||
		family.Group != 0 && family.Group != 2 && family.Group != 3 && family.Group != 5 {
		return 0, false, nil
	}
	if err := ctx.Err(); err != nil {
		return 0, false, err
	}
	source, ok, err := shortSourceCells(ctx, proof, family)
	if err != nil || !ok {
		return 0, false, err
	}
	const edgePieces = 4
	mesh := make([][3]loftFaceInterval, 0, edgePieces*(len(held)-1))
	for i := 1; i < len(held); i++ {
		if !proofbound.FiniteVec(held[i-1]) || !proofbound.FiniteVec(held[i]) ||
			held[i-1] == held[i] {
			return 0, false, nil
		}
		a := [3]float64{held[i-1].X, held[i-1].Y, held[i-1].Z}
		z := [3]float64{held[i].X, held[i].Y, held[i].Z}
		for piece := range edgePieces {
			var box [3]loftFaceInterval
			for axis := range box {
				left := shortRatAdd(seamRatF(a[axis]), shortRatMul(shortRatSub(seamRatF(z[axis]), seamRatF(a[axis])),
					big.NewRat(int64(piece), edgePieces)))
				right := shortRatAdd(seamRatF(a[axis]), shortRatMul(shortRatSub(seamRatF(z[axis]), seamRatF(a[axis])),
					big.NewRat(int64(piece+1), edgePieces)))
				if left.Cmp(right) > 0 {
					left, right = right, left
				}
				box[axis] = loftFaceInterval{Lo: left, Hi: right}
			}
			mesh = append(mesh, box)
		}
	}
	// Most matching boxes are much closer than minBound. Outward float boxes
	// can certify those pairs without repeating large rational subtractions
	// for every tooth copy. Uncertain cases retain the exact search below.
	if fast, accepted := ShortFloatCover(ctx, source, mesh, minBound); accepted {
		return fast, true, nil
	}
	limit := shortRatMul(seamRatF(minBound), seamRatF(minBound))
	var largest *big.Rat
	for direction := range 2 {
		from, to := source, mesh
		if direction == 1 {
			from, to = mesh, source
		}
		for i, a := range from {
			if i%4 == 0 {
				if err := ctx.Err(); err != nil {
					return 0, false, err
				}
			}
			var nearest *big.Rat
			for _, b := range to {
				distance, valid := ShortBoxesDistanceSquared(a, b)
				if !valid {
					return 0, false, nil
				}
				if nearest == nil || distance.Cmp(nearest) < 0 {
					nearest = distance
				}
			}
			if nearest == nil || nearest.Cmp(limit) >= 0 {
				if nearest == nil {
					return 0, false, nil
				}
				return proofbound.RatSqrtUp(nearest), false, nil
			}
			if largest == nil || nearest.Cmp(largest) > 0 {
				largest = nearest
			}
		}
	}
	if err := ctx.Err(); err != nil {
		return 0, false, err
	}
	bound := proofbound.RatSqrtUp(largest)
	return bound, !proofbound.IsNonFinite(bound) && bound < minBound, nil
}

func shortSourceCells(ctx context.Context, proof *facetedrecord.SpiralFirstJoinSource,
	family ShortCapFamily) ([][3]loftFaceInterval, bool, error) {
	part := family.Pieces[0]
	if len(part.Span) != 2 && len(part.Span) != 4 || len(part.Q[0]) == 0 {
		return nil, false, nil
	}
	start, end := new(big.Rat), shortRatOne()
	if len(part.Span) == 4 {
		clip := family.FiniteClip
		if clip.Band != family.Band || clip.SLo == nil || clip.SHi == nil ||
			clip.SLo.Sign() <= 0 || clip.SLo.Cmp(clip.SHi) >= 0 ||
			clip.SHi.Cmp(shortRatOne()) >= 0 || family.FiniteEnd < 0 || family.FiniteEnd > 1 {
			return nil, false, nil
		}
		if family.FiniteEnd == 0 {
			end = clip.SHi
		} else {
			start = clip.SLo
		}
	}
	width := shortRatDiv(shortRatSub(end, start), big.NewRat(16, 1))
	first, second := proof.Tooth.Maps[family.Band], proof.Tooth.Maps[family.Band+1]
	boxes := make([][3]loftFaceInterval, 0, 16)
	for cell := range 16 {
		if cell%4 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, false, err
			}
		}
		lo := shortRatAdd(start, shortRatMul(big.NewRat(int64(cell), 1), width))
		hi := shortRatAdd(lo, width)
		if len(part.Span) == 4 && (family.FiniteEnd == 0 && cell == 15 && lo.Cmp(family.FiniteClip.SLo) >= 0 ||
			family.FiniteEnd == 1 && cell == 0 && hi.Cmp(family.FiniteClip.SHi) <= 0) {
			// The one enlarged clip cell must still contain a real, finite
			// source arc; it cannot be only the excluded side of the cap.
			return nil, false, nil
		}
		var restricted [3][]*big.Rat
		for row := range restricted {
			restricted[row] = polynomial.BernsteinRestrict(part.Q[row], lo, hi)
		}
		t, yes := ShortCapTBracket(restricted)
		if !yes {
			return nil, false, nil
		}
		box, yes := loftface.JunctionBox(first, second, part.Span, lo, hi, t.Lo, t.Hi)
		if !yes {
			return nil, false, nil
		}
		boxes = append(boxes, box)
	}
	return boxes, true, nil
}
