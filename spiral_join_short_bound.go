package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// spiralShortHeldBound compares an authenticated finite cap intersection
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
func spiralShortHeldBound(ctx context.Context, proof *spiralFirstJoinSource,
	family spiralShortCapFamily, held []r3.Vec, minBound float64) (float64, bool, error) {
	if ctx == nil || proof == nil || proof.tooth == nil || len(proof.tooth.maps) != 9 ||
		len(family.pieces) != 1 || len(held) < 2 || len(held) > 1025 ||
		proofbound.IsNonFinite(minBound) || minBound <= 0 ||
		family.band != 1 && family.band != 7 ||
		family.group != 0 && family.group != 2 && family.group != 3 && family.group != 5 {
		return 0, false, nil
	}
	if err := ctx.Err(); err != nil {
		return 0, false, err
	}
	source, ok, err := spiralShortSourceCells(ctx, proof, family)
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
				left := lfAdd(lfF(a[axis]), lfMul(lfSub(lfF(z[axis]), lfF(a[axis])),
					big.NewRat(int64(piece), edgePieces)))
				right := lfAdd(lfF(a[axis]), lfMul(lfSub(lfF(z[axis]), lfF(a[axis])),
					big.NewRat(int64(piece+1), edgePieces)))
				if left.Cmp(right) > 0 {
					left, right = right, left
				}
				box[axis] = loftFaceInterval{lo: left, hi: right}
			}
			mesh = append(mesh, box)
		}
	}
	limit := lfMul(lfF(minBound), lfF(minBound))
	var largest *big.Rat
	for direction := 0; direction < 2; direction++ {
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
				distance, valid := spiralShortBoxesDistanceSquared(a, b)
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

func spiralShortSourceCells(ctx context.Context, proof *spiralFirstJoinSource,
	family spiralShortCapFamily) ([][3]loftFaceInterval, bool, error) {
	part := family.pieces[0]
	if len(part.span) != 2 && len(part.span) != 4 || len(part.q[0]) == 0 {
		return nil, false, nil
	}
	start, end := lfZero(), lfOne()
	if len(part.span) == 4 {
		clip := family.finiteClip
		if clip.band != family.band || clip.sLo == nil || clip.sHi == nil ||
			clip.sLo.Sign() <= 0 || clip.sLo.Cmp(clip.sHi) >= 0 ||
			clip.sHi.Cmp(lfOne()) >= 0 || family.finiteEnd < 0 || family.finiteEnd > 1 {
			return nil, false, nil
		}
		if family.finiteEnd == 0 {
			end = clip.sHi
		} else {
			start = clip.sLo
		}
	}
	width := lfDiv(lfSub(end, start), big.NewRat(16, 1))
	first, second := proof.tooth.maps[family.band], proof.tooth.maps[family.band+1]
	boxes := make([][3]loftFaceInterval, 0, 16)
	for cell := range 16 {
		if cell%4 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, false, err
			}
		}
		lo := lfAdd(start, lfMul(big.NewRat(int64(cell), 1), width))
		hi := lfAdd(lo, width)
		if len(part.span) == 4 && (family.finiteEnd == 0 && cell == 15 && lo.Cmp(family.finiteClip.sLo) >= 0 ||
			family.finiteEnd == 1 && cell == 0 && hi.Cmp(family.finiteClip.sHi) <= 0) {
			// The one enlarged clip cell must still contain a real, finite
			// source arc; it cannot be only the excluded side of the cap.
			return nil, false, nil
		}
		var restricted [3][]*big.Rat
		for row := range restricted {
			restricted[row] = spiralBernRestrict(part.q[row], lo, hi)
		}
		t, yes := spiralShortCapTBracket(restricted)
		if !yes {
			return nil, false, nil
		}
		box, yes := spiralJunctionBox(first, second, part.span, lo, hi, t.lo, t.hi)
		if !yes {
			return nil, false, nil
		}
		boxes = append(boxes, box)
	}
	return boxes, true, nil
}

func spiralShortBoxesDistanceSquared(a, b [3]loftFaceInterval) (*big.Rat, bool) {
	distance := new(big.Rat)
	for axis := range a {
		if a[axis].lo == nil || a[axis].hi == nil || b[axis].lo == nil || b[axis].hi == nil ||
			a[axis].lo.Cmp(a[axis].hi) > 0 || b[axis].lo.Cmp(b[axis].hi) > 0 {
			return nil, false
		}
		lo := lfAbs(lfSub(a[axis].lo, b[axis].hi))
		hi := lfAbs(lfSub(a[axis].hi, b[axis].lo))
		if hi.Cmp(lo) > 0 {
			lo = hi
		}
		distance = lfAdd(distance, lfMul(lo, lo))
	}
	return distance, true
}
