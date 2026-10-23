package loftface

import (
	"math/big"
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
