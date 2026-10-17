package loftface

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// VerticalLimit is one radial limit of a vertical source cross section.
type VerticalLimit struct {
	// Side is the sign of radius squared minus the limit squared when there
	// is no root. A root has strict signs on both sides of its bracket.
	Side, Before, After int
	Root                Interval
	Box                 [3]Interval
}

// VerticalPiece is one retained source interval of a vertical cross section.
type VerticalPiece struct {
	Source         Interval
	LoOpen, HiOpen bool // equality with either radial limit excludes the endpoint
	Box            [3]Interval
}

// At the certified band time, Y/Z are an orientation-preserving affine map
// of the live Sketch U/V coordinates when both cap denominators are constant.
// A nil record makes no claim about angular order or similarity.
type VerticalAffine struct {
	Offset [2]*big.Rat
	Column [2][2]*big.Rat // [source U or V][world Y or Z]
	Det    *big.Rat
}

// VerticalTraceBoxDistance bounds a held point's distance from an exact
// vertical-trace source box.
func VerticalTraceBoxDistance(box [3]Interval, held r3.Vec) (float64, bool) {
	coordinates := [3]float64{held.X, held.Y, held.Z}
	distance := ratZero()
	for axis, coordinate := range coordinates {
		interval := box[axis]
		if interval.Lo == nil || interval.Hi == nil || interval.Lo.Cmp(interval.Hi) > 0 {
			return 0, false
		}
		lo := ratAbs(ratSub(ratF(coordinate), interval.Lo))
		hi := ratAbs(ratSub(ratF(coordinate), interval.Hi))
		if lo.Cmp(hi) > 0 {
			distance = ratAdd(distance, lo)
		} else {
			distance = ratAdd(distance, hi)
		}
	}
	bound := proofbound.RatFloatUp(distance)
	return bound, !proofbound.IsNonFinite(bound) && bound >= 0
}

// ConstantXBand returns the band, and the exact time in it, at which the
// constant-X cap maps reach x.
func ConstantXBand(maps []Map, box [4]*big.Rat,
	x *big.Rat) (int, *big.Rat, bool) {
	if len(maps) < 2 || x == nil {
		return -1, nil, false
	}
	capX := make([]*big.Rat, len(maps))
	for i, mapped := range maps {
		if PositiveDenominator(mapped, box) != nil {
			return -1, nil, false
		}
		mappedX, ok := ConstantX(mapped)
		capX[i] = mappedX
		if !ok || i > 0 && capX[i].Cmp(capX[i-1]) <= 0 || capX[i].Cmp(x) == 0 {
			return -1, nil, false
		}
	}
	band := -1
	for i := 0; i+1 < len(capX); i++ {
		if capX[i].Cmp(x) < 0 && capX[i+1].Cmp(x) > 0 {
			band = i
			break
		}
	}
	if band < 0 {
		return -1, nil, false
	}
	time := ratDiv(ratSub(x, capX[band]), ratSub(capX[band+1], capX[band]))
	return band, time, true
}

// A linear-fractional X map is constant on the entire source rectangle
// exactly when its numerator is that constant times its denominator.
func ConstantX(mapped Map) (*big.Rat, bool) {
	if mapped.G0 == nil || mapped.G0.Sign() <= 0 || mapped.Constant[0] == nil ||
		mapped.Q0[0] == nil || mapped.QU[0] == nil || mapped.QV[0] == nil ||
		mapped.GU == nil || mapped.GV == nil {
		return nil, false
	}
	ratio := ratDiv(mapped.Q0[0], mapped.G0)
	if mapped.QU[0].Cmp(ratMul(ratio, mapped.GU)) != 0 ||
		mapped.QV[0].Cmp(ratMul(ratio, mapped.GV)) != 0 {
		return nil, false
	}
	return ratAdd(mapped.Constant[0], ratio), true
}

// VerticalAffineMap returns the affine map from source U/V to world Y/Z at
// the given band time, or false when it is not an orientation-preserving map.
func VerticalAffineMap(first, last Map,
	time *big.Rat) (*VerticalAffine, bool) {
	if time == nil || time.Sign() < 0 || time.Cmp(ratOne()) > 0 {
		return nil, false
	}
	var mappedCaps [2]VerticalAffine
	for i, mapped := range [...]Map{first, last} {
		if mapped.G0 == nil || mapped.G0.Sign() <= 0 ||
			mapped.GU == nil || mapped.GU.Sign() != 0 ||
			mapped.GV == nil || mapped.GV.Sign() != 0 {
			return nil, false
		}
		for axis := range mappedCaps[i].Offset {
			worldAxis := axis + 1
			if mapped.Constant[worldAxis] == nil || mapped.Q0[worldAxis] == nil ||
				mapped.QU[worldAxis] == nil || mapped.QV[worldAxis] == nil {
				return nil, false
			}
			mappedCaps[i].Offset[axis] = ratAdd(mapped.Constant[worldAxis],
				ratDiv(mapped.Q0[worldAxis], mapped.G0))
			mappedCaps[i].Column[0][axis] = ratDiv(mapped.QU[worldAxis], mapped.G0)
			mappedCaps[i].Column[1][axis] = ratDiv(mapped.QV[worldAxis], mapped.G0)
		}
	}
	blend := func(a, b *big.Rat) *big.Rat {
		return ratAdd(ratMul(ratSub(ratOne(), time), a), ratMul(time, b))
	}
	result := &VerticalAffine{}
	for axis := range result.Offset {
		result.Offset[axis] = blend(mappedCaps[0].Offset[axis], mappedCaps[1].Offset[axis])
		for sourceAxis := range result.Column {
			result.Column[sourceAxis][axis] = blend(mappedCaps[0].Column[sourceAxis][axis],
				mappedCaps[1].Column[sourceAxis][axis])
		}
	}
	result.Det = ratSub(ratMul(result.Column[0][0], result.Column[1][1]),
		ratMul(result.Column[0][1], result.Column[1][0]))
	if result.Det.Sign() <= 0 {
		return nil, false
	}
	return result, true
}

// Bernstein control signs prove a strict side. Strict endpoint and
// derivative signs prove one transverse root and exclude all others.
func VerticalClassify(q []*big.Rat) (VerticalLimit, bool) {
	var result VerticalLimit
	if len(q) < 2 || q[0].Sign() == 0 || q[len(q)-1].Sign() == 0 {
		return result, false
	}
	positive, negative := true, true
	for _, coefficient := range q {
		positive = positive && coefficient.Sign() > 0
		negative = negative && coefficient.Sign() < 0
	}
	if positive {
		result.Side = 1
		return result, true
	}
	if negative {
		result.Side = -1
		return result, true
	}
	signed := q
	if q[0].Sign() > 0 {
		signed = NegateRow(q)
	}
	if !polynomial.BernsteinRowsCross([3][]*big.Rat{signed, signed, signed}) {
		return result, false
	}
	root := IsolateRootN(signed, 80)
	if root.Lo.Sign() <= 0 || root.Hi.Cmp(ratOne()) >= 0 {
		return result, false
	}
	result.Root = Interval{Lo: root.Lo, Hi: root.Hi}
	result.Before, result.After = q[0].Sign(), q[len(q)-1].Sign()
	return result, true
}

// VerticalPieces returns the retained source pieces between two radial limits.
func VerticalPieces(first, last Map,
	span freeform.BezierSpan, time *big.Rat, loQ, hiQ []*big.Rat,
	loRoot, hiRoot Interval) ([]VerticalPiece, bool) {
	if loRoot.Lo != nil && hiRoot.Lo != nil &&
		loRoot.Lo.Cmp(hiRoot.Hi) <= 0 && hiRoot.Lo.Cmp(loRoot.Hi) <= 0 {
		return nil, false
	}
	endpoints := []*big.Rat{ratZero(), ratOne()}
	for _, root := range [...]Interval{loRoot, hiRoot} {
		if root.Lo != nil {
			endpoints = append(endpoints, root.Lo, root.Hi)
		}
	}
	sortRats(endpoints)
	for i := 1; i < len(endpoints); i++ {
		if endpoints[i].Cmp(endpoints[i-1]) == 0 {
			return nil, false
		}
	}
	var pieces []VerticalPiece
	for i := 0; i+1 < len(endpoints); i++ {
		a, z := endpoints[i], endpoints[i+1]
		if verticalWithinRoot(a, z, loRoot) || verticalWithinRoot(a, z, hiRoot) {
			continue
		}
		mid := ratMul(ratAdd(a, z), big.NewRat(1, 2))
		if polynomial.BernsteinSign(loQ, mid) <= 0 || polynomial.BernsteinSign(hiQ, mid) >= 0 {
			continue
		}
		box, ok := JunctionBox(first, last, span, a, z, time, time)
		if !ok {
			return nil, false
		}
		loLower, loUpper := polynomial.BernsteinSign(loQ, a), polynomial.BernsteinSign(hiQ, a)
		hiLower, hiUpper := polynomial.BernsteinSign(loQ, z), polynomial.BernsteinSign(hiQ, z)
		if loLower < 0 || loUpper > 0 || hiLower < 0 || hiUpper > 0 {
			return nil, false
		}
		pieces = append(pieces, VerticalPiece{
			Source: Interval{Lo: a, Hi: z},
			LoOpen: loLower == 0 || loUpper == 0,
			HiOpen: hiLower == 0 || hiUpper == 0,
			Box:    box})
	}
	return pieces, true
}

// verticalWithinRoot reports whether [a, z] lies inside the root bracket.
func verticalWithinRoot(a, z *big.Rat, root Interval) bool {
	return root.Lo != nil && a.Cmp(root.Lo) >= 0 && z.Cmp(root.Hi) <= 0
}

// sortRats sorts values ascending by insertion.
func sortRats(values []*big.Rat) {
	for i := 1; i < len(values); i++ {
		for j := i; j > 0 && values[j].Cmp(values[j-1]) < 0; j-- {
			values[j], values[j-1] = values[j-1], values[j]
		}
	}
}

// CircleOutsideMaps reports whether every cap map keeps the circle of the
// given radius at x outside its source box.
func CircleOutsideMaps(maps []Map, box [4]*big.Rat,
	x, radius *big.Rat) bool {
	if len(maps) < 2 || x == nil || radius == nil || radius.Sign() <= 0 {
		return false
	}
	corners := BoxCorners(box)
	for _, mapped := range maps {
		if PositiveDenominator(mapped, box) != nil {
			return false
		}
	}
	radius2 := ratMul(radius, radius)
	// Each map is evaluated at most once, when the first band that needs it
	// is reached, so an early refusal never costs an evaluation.
	ranges := make([]*[3]Interval, len(maps))
	rangeOf := func(i int) *[3]Interval {
		if ranges[i] == nil {
			r := cornerAxisRanges(maps[i], corners)
			ranges[i] = &r
		}
		return ranges[i]
	}
	for band := 0; band+1 < len(maps); band++ {
		first, second := rangeOf(band), rangeOf(band+1)
		var range3 [3]Interval
		for coordinate := range range3 {
			range3[coordinate] = IntervalHull(first[coordinate], second[coordinate])
		}
		if x.Cmp(range3[0].Lo) < 0 || x.Cmp(range3[0].Hi) > 0 {
			continue
		}
		minimum := ratAdd(IntervalSquareMin(range3[1]), IntervalSquareMin(range3[2]))
		maximum := ratAdd(IntervalSquareMax(range3[1]), IntervalSquareMax(range3[2]))
		if radius2.Cmp(minimum) >= 0 && radius2.Cmp(maximum) <= 0 {
			return false
		}
	}
	return true
}
