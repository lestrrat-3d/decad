package spherepath

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// QuadraticAt evaluates the exact squared sphere separation polynomial.
func QuadraticAt(a, b, c proofarith.Dyadic, f *big.Rat) *big.Rat {
	out := new(big.Rat).Mul(a.Rat(), f)
	out.Add(out, b.Rat())
	out.Mul(out, f)
	return out.Add(out, c.Rat())
}

func ratFloatNearest(v *big.Rat) float64 { f, _ := v.Float64(); return f }

// ImpactBracket leaves a representable positive-overlap margin at the right
// endpoint of a source-sphere impact.
func ImpactBracket(root, duration, resolution *big.Rat) (*big.Rat, *big.Rat, bool) {
	grid := big.NewInt(1)
	four := big.NewRat(4, 1)
	for range 61 {
		width := new(big.Rat).Quo(duration, new(big.Rat).SetInt(grid))
		if new(big.Rat).Mul(width, four).Cmp(resolution) <= 0 {
			scaled := new(big.Rat).Mul(root, new(big.Rat).SetInt(grid))
			floor := new(big.Int).Quo(scaled.Num(), scaled.Denom())
			leftIdx := new(big.Int).Sub(new(big.Int).Set(floor), big.NewInt(1))
			rightIdx := new(big.Int).Add(new(big.Int).Set(floor), big.NewInt(2))
			left := new(big.Rat).SetFrac(leftIdx, grid)
			right := new(big.Rat).SetFrac(rightIdx, grid)
			if right.Cmp(big.NewRat(1, 1)) > 0 && root.Cmp(big.NewRat(1, 1)) < 0 {
				// A prefix ending just after impact can use its real endpoint.
				// The sampled manifold below still has to prove overlap there.
				right = big.NewRat(1, 1)
			}
			span := new(big.Rat).Mul(new(big.Rat).Sub(right, left), duration)
			if left.Sign() <= 0 || right.Cmp(big.NewRat(1, 1)) > 0 ||
				span.Cmp(resolution) > 0 ||
				proofarith.FloatRat(ratFloatNearest(left)).Cmp(left) != 0 ||
				proofarith.FloatRat(ratFloatNearest(right)).Cmp(right) != 0 {
				return nil, nil, false
			}
			return left, right, true
		}
		grid.Lsh(grid, 1)
	}
	return nil, nil, false
}

// PairImpactBracket also permits an impact at the exact final instant.
// Its left dyadic endpoint must still lie strictly before the support root.
func PairImpactBracket(root, duration, resolution *big.Rat) (*big.Rat, *big.Rat, bool) {
	left, right, ok := ImpactBracket(root, duration, resolution)
	if ok {
		return left, right, true
	}
	one := big.NewRat(1, 1)
	if root.Sign() <= 0 || root.Cmp(one) > 0 {
		return nil, nil, false
	}
	grid := big.NewInt(1)
	for range 61 {
		width := new(big.Rat).Quo(duration, new(big.Rat).SetInt(grid))
		if width.Cmp(resolution) <= 0 {
			left = new(big.Rat).Sub(one, new(big.Rat).SetFrac(big.NewInt(1), grid))
			if left.Sign() > 0 && left.Cmp(root) < 0 &&
				proofarith.FloatRat(ratFloatNearest(left)).Cmp(left) == 0 {
				return left, one, true
			}
			if left.Cmp(root) >= 0 {
				return nil, nil, false
			}
		}
		grid.Lsh(grid, 1)
	}
	return nil, nil, false
}

// QuadraticBracket searches only the decreasing side of the exact squared
// distance, with a right seed already at or inside first contact.
func QuadraticBracket(a, b, c proofarith.Dyadic, vertex, duration, resolution *big.Rat) (
	*big.Rat, *big.Rat, bool) {
	zero, one := new(big.Rat), big.NewRat(1, 1)
	right := new(big.Rat).Set(one)
	if QuadraticAt(a, b, c, one).Sign() > 0 {
		found := false
		grid := big.NewInt(1)
		for range 61 {
			index := new(big.Int).Quo(new(big.Int).Mul(vertex.Num(), grid), vertex.Denom())
			candidate := new(big.Rat).SetFrac(index, grid)
			if candidate.Sign() > 0 && candidate.Cmp(one) < 0 &&
				QuadraticAt(a, b, c, candidate).Sign() <= 0 {
				right, found = candidate, true
				break
			}
			grid.Lsh(grid, 1)
		}
		if !found {
			return nil, nil, false
		}
	}
	left := zero
	for range 61 {
		width := new(big.Rat).Sub(right, left)
		span := new(big.Rat).Mul(width, duration)
		if new(big.Rat).Mul(span, big.NewRat(4, 1)).Cmp(resolution) <= 0 {
			before := new(big.Rat).Sub(left, width)
			after := new(big.Rat).Add(right, width)
			if after.Cmp(one) > 0 {
				after = one
			}
			if before.Sign() > 0 && QuadraticAt(a, b, c, before).Sign() > 0 &&
				QuadraticAt(a, b, c, after).Sign() <= 0 &&
				proofarith.FloatRat(ratFloatNearest(before)).Cmp(before) == 0 &&
				proofarith.FloatRat(ratFloatNearest(after)).Cmp(after) == 0 {
				return before, after, true
			}
		}
		mid := new(big.Rat).Add(left, right)
		mid.Quo(mid, big.NewRat(2, 1))
		if QuadraticAt(a, b, c, mid).Sign() > 0 {
			left = mid
		} else {
			right = mid
		}
	}
	return nil, nil, false
}
