package box

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// PoseDeviation bounds the L1 distance between a float-pose source box and
// the ideal affine box. Each support endpoint is compared as an exact rational.
func PoseDeviation(start, observed AxisBox, delta [3]proofarith.Dyadic, f *big.Rat) *big.Rat {
	total := new(big.Rat)
	for i := range 3 {
		move := new(big.Rat).Mul(delta[i].Rat(), f)
		idealLo := new(big.Rat).Add(start.Lo[i].Rat(), move)
		idealHi := new(big.Rat).Add(start.Hi[i].Rat(), move)
		lo := new(big.Rat).Sub(observed.Lo[i].Rat(), idealLo)
		hi := new(big.Rat).Sub(observed.Hi[i].Rat(), idealHi)
		lo.Abs(lo)
		hi.Abs(hi)
		if hi.Cmp(lo) > 0 {
			lo = hi
		}
		total.Add(total, lo)
	}
	return total
}

// TranslatePair moves both source boxes by the same exact path fraction.
func TranslatePair(a, b AxisBox, da, db [3]proofarith.Dyadic, f *big.Rat) (AxisBox, AxisBox, bool) {
	fraction, ok := proofarith.DyOfRat(f)
	if !ok {
		return AxisBox{}, AxisBox{}, false
	}
	for i := range 3 {
		moveA, moveB := proofarith.DyMul(da[i], fraction), proofarith.DyMul(db[i], fraction)
		a.Lo[i], a.Hi[i] = proofarith.DyAdd(a.Lo[i], moveA), proofarith.DyAdd(a.Hi[i], moveA)
		b.Lo[i], b.Hi[i] = proofarith.DyAdd(b.Lo[i], moveB), proofarith.DyAdd(b.Hi[i], moveB)
	}
	return a, b, true
}

// TranslateOriented moves an oriented source box by delta·fraction. Each
// axis step must be an exact dyadic; ok is false when one is not.
func TranslateOriented(box OrientedBox, delta [3]proofarith.Dyadic, fraction *big.Rat) (OrientedBox, bool) {
	for axis := range 3 {
		step, ok := proofarith.DyOfRat(new(big.Rat).Mul(delta[axis].Rat(), fraction))
		if !ok {
			return OrientedBox{}, false
		}
		for corner := range box.Corner {
			box.Corner[corner][axis] = proofarith.DyAdd(box.Corner[corner][axis], step)
		}
	}
	return box, true
}

// OrientedIntervalSeparated reports whether one of a's face normals strictly
// separates the two oriented boxes at both ends of the fraction span
// [from, to], each box translated by its delta. Translation keeps both
// boxes' edges, so each projection moves affinely in the fraction and an
// exact positive support gap at both ends is positive throughout the span.
// It reports false when a translation is not an exact dyadic.
func OrientedIntervalSeparated(a, b OrientedBox, da, db [3]proofarith.Dyadic, from, to *big.Rat) bool {
	a0, okA := TranslateOriented(a, da, from)
	b0, okB := TranslateOriented(b, db, from)
	a1, okC := TranslateOriented(a, da, to)
	b1, okD := TranslateOriented(b, db, to)
	if !okA || !okB || !okC || !okD {
		return false
	}
	for axis := range 3 {
		i, j := (axis+1)%3, (axis+2)%3
		normal := proofarith.DvCross(a.Edge[i], a.Edge[j])
		if proofarith.DvIsZero(normal) {
			continue
		}
		alo0, ahi0 := OrientedProjection(a0, normal)
		blo0, bhi0 := OrientedProjection(b0, normal)
		alo1, ahi1 := OrientedProjection(a1, normal)
		blo1, bhi1 := OrientedProjection(b1, normal)
		if proofarith.DyCmp(ahi0, blo0) < 0 && proofarith.DyCmp(ahi1, blo1) < 0 ||
			proofarith.DyCmp(bhi0, alo0) < 0 && proofarith.DyCmp(bhi1, alo1) < 0 {
			return true
		}
	}
	return false
}

// AffineEqualityRoot finds when two moving support endpoints coincide.
func AffineEqualityRoot(a, da, b, db proofarith.Dyadic) *big.Rat {
	delta := proofarith.DySubScalar(da, db)
	if delta.IsZero() {
		return nil
	}
	return new(big.Rat).Quo(proofarith.DySubScalar(b, a).Rat(), delta.Rat())
}

// AffineEqualityRootWithin reports an endpoint owner change after the start.
func AffineEqualityRootWithin(a, da, b, db proofarith.Dyadic, end *big.Rat) bool {
	root := AffineEqualityRoot(a, da, b, db)
	return root != nil && root.Sign() > 0 && root.Cmp(end) <= 0
}

// TransitionRoot finds the first projected patch-owner or edge-contact change.
// Roots at zero use the right-sided patch.
func TransitionRoot(a, b AxisBox, da, db [3]proofarith.Dyadic, axis int) *big.Rat {
	var earliest *big.Rat
	for i := range 3 {
		if i == axis {
			continue
		}
		for _, pair := range [][4]proofarith.Dyadic{
			{a.Lo[i], da[i], b.Lo[i], db[i]},
			{a.Hi[i], da[i], b.Hi[i], db[i]},
			{a.Lo[i], da[i], b.Hi[i], db[i]},
			{b.Lo[i], db[i], a.Hi[i], da[i]},
		} {
			root := AffineEqualityRoot(pair[0], pair[1], pair[2], pair[3])
			if root == nil || root.Sign() <= 0 || root.Cmp(big.NewRat(1, 1)) > 0 {
				continue
			}
			if earliest == nil || root.Cmp(earliest) < 0 {
				earliest = root
			}
		}
	}
	return earliest
}

// ContactRootBracket encloses an exact event root on a representable dyadic grid.
func ContactRootBracket(root, duration, resolution *big.Rat) (*big.Rat, *big.Rat, bool) {
	grid := big.NewInt(1)
	for range 61 {
		width := new(big.Rat).Quo(duration, new(big.Rat).SetInt(grid))
		if width.Cmp(resolution) <= 0 {
			scaled := new(big.Rat).Mul(root, new(big.Rat).SetInt(grid))
			leftIdx := new(big.Int).Quo(scaled.Num(), scaled.Denom())
			rightIdx := new(big.Int).Add(new(big.Int).Set(leftIdx), big.NewInt(1))
			if scaled.IsInt() {
				leftIdx.Sub(leftIdx, big.NewInt(1))
				rightIdx.Sub(rightIdx, big.NewInt(1))
			}
			left := new(big.Rat).SetFrac(leftIdx, grid)
			right := new(big.Rat).SetFrac(rightIdx, grid)
			leftFloat, _ := left.Float64()
			rightFloat, _ := right.Float64()
			if left.Sign() < 0 || right.Cmp(big.NewRat(1, 1)) > 0 ||
				proofarith.FloatRat(leftFloat).Cmp(left) != 0 ||
				proofarith.FloatRat(rightFloat).Cmp(right) != 0 {
				return nil, nil, false
			}
			return left, right, true
		}
		grid.Lsh(grid, 1)
	}
	return nil, nil, false
}

// TrackPointsWithin bounds conversion of any contact coordinate in the
// start/end box endpoint envelope by one ULP at its maximum magnitude.
func TrackPointsWithin(a, b AxisBox, da, db [3]proofarith.Dyadic, resolution float64) bool {
	maximum := new(big.Rat)
	for _, moving := range []struct {
		box   AxisBox
		delta [3]proofarith.Dyadic
	}{{a, da}, {b, db}} {
		for i := range 3 {
			for _, endpoint := range []proofarith.Dyadic{moving.box.Lo[i], moving.box.Hi[i]} {
				for _, value := range []proofarith.Dyadic{endpoint, proofarith.DyAdd(endpoint, moving.delta[i])} {
					abs := new(big.Rat).Abs(value.Rat())
					if abs.Cmp(maximum) > 0 {
						maximum = abs
					}
				}
			}
		}
	}
	maxFloat := proofbound.RatFloatUp(maximum)
	if !finite(maxFloat) {
		return false
	}
	ulp := math.Nextafter(maxFloat, math.Inf(1)) - maxFloat
	bound := proofbound.Radius3D(ulp)
	return finite(bound) && bound <= resolution
}

// RelationDistanceWithin bounds the least support gap or penetration depth
// directly from exact axis-aligned box endpoints.
func RelationDistanceWithin(a, b AxisBox, limit *big.Rat) bool {
	var distance *big.Rat
	for axis := range 3 {
		for _, candidate := range []*big.Rat{
			new(big.Rat).Abs(new(big.Rat).Sub(a.Hi[axis].Rat(), b.Lo[axis].Rat())),
			new(big.Rat).Abs(new(big.Rat).Sub(b.Hi[axis].Rat(), a.Lo[axis].Rat())),
		} {
			if distance == nil || candidate.Cmp(distance) < 0 {
				distance = candidate
			}
		}
	}
	return distance != nil && distance.Cmp(limit) <= 0
}
