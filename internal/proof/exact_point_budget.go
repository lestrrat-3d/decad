package proof

import (
	"fmt"
	"math"
	"math/bits"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/r3"
)

// XptOfBounded reserves an upper bound for the binary64 lift's independent
// limb payloads in 64-byte blocks before constructing the exact point.
// The float significands fit the initial three-word slots. Each of the four
// later shifts can grow once; the bound includes four spare words per growth
// and the temporary one-word denominator input. It is deliberately an upper
// bound even when a coordinate is zero or stays inline.
func XptOfBounded(v r3.Vec, reserve func(uint64) error) (Xpt, error) {
	if reserve == nil {
		return Xpt{}, fmt.Errorf("%w: exact lift needs a reservation", decaderr.ErrUnsupported)
	}
	base := 0
	var exps [3]int
	var lengths [3]int
	for i, f := range [3]float64{v.X, v.Y, v.Z} {
		if math.IsNaN(f) || math.IsInf(f, 0) {
			return Xpt{}, fmt.Errorf("%w: exact lift needs finite coordinates", decaderr.ErrUnsupported)
		}
		raw := math.Float64bits(f)
		fraction := raw & ((uint64(1) << 52) - 1)
		field := int((raw >> 52) & 0x7ff)
		if field == 0 {
			exps[i] = -1074
			lengths[i] = bits.Len64(fraction)
		} else {
			exps[i] = field - 1023 - 52
			lengths[i] = 53
		}
		if lengths[i] != 0 && exps[i] < base {
			base = exps[i]
		}
	}
	maxBits := 1 - base
	for i := range exps {
		if lengths[i] != 0 {
			maxBits = max(maxBits, lengths[i]+exps[i]-base)
		}
	}
	words := (maxBits + bits.UintSize - 1) / bits.UintSize
	bytes := (words + 4) * (bits.UintSize / 8)
	blocks := uint64((bytes + 63) / 64)
	// Four independently grown coordinates, the initial shared word storage,
	// and a one-word temporary used to form the denominator.
	if err := reserve(4*blocks + 3); err != nil {
		return Xpt{}, err
	}
	return XptOf(v), nil
}

// TriangleNormalBounded precharges the exact dyadic normal and its temporary
// limb payloads before evaluation. A span over 120 bits declines before
// multiplication; within that span the existing small-integer arithmetic has
// a fixed number of payloads and does not enter large-number multiplication.
func TriangleNormalBounded(a, b, c r3.Vec, reserve func(uint64) error) (Xpt, error) {
	if reserve == nil {
		return Xpt{}, fmt.Errorf("%w: exact normal needs a reservation", decaderr.ErrUnsupported)
	}
	base, top := 0, 0
	found := false
	for _, v := range [3]r3.Vec{a, b, c} {
		for _, f := range [3]float64{v.X, v.Y, v.Z} {
			if math.IsNaN(f) || math.IsInf(f, 0) {
				return Xpt{}, fmt.Errorf("%w: exact normal needs finite coordinates", decaderr.ErrUnsupported)
			}
			raw := math.Float64bits(f)
			fraction := raw & ((uint64(1) << 52) - 1)
			field := int((raw >> 52) & 0x7ff)
			exp, length := -1074, bits.Len64(fraction)
			if field != 0 {
				exp = field - 1023 - 52
				length = 53
			}
			if length == 0 {
				continue
			}
			if !found {
				base, top, found = exp, exp+length, true
			} else {
				base = min(base, exp)
				top = max(top, exp+length)
			}
		}
	}
	if !found {
		return Xpt{}, fmt.Errorf("%w: exact normal has no extent", decaderr.ErrUnsupported)
	}
	span := top - base
	if span > 120 {
		return Xpt{}, fmt.Errorf("%w: exact normal exceeds the bounded dyadic span", decaderr.ErrUnsupported)
	}
	// Each aligned difference fits span+1 bits and each cross component fits
	// 2*span+3. Include the homogeneous denominator and shifted coordinates.
	normalBase := min(0, 2*base)
	maxBits := max(2*span+3, 1-normalBase, 2*top+3-normalBase)
	words := (maxBits + bits.UintSize - 1) / bits.UintSize
	blocks := uint64(((words+4)*(bits.UintSize/8) + 63) / 64)
	// Six differences, six products, three final differences and the output
	// use fewer than 80 independently backed integer payloads in this span.
	if err := reserve(80*blocks + 4); err != nil {
		return Xpt{}, err
	}
	da, db, dc := DyVec(a), DyVec(b), DyVec(c)
	d := DvCross(DvSub(db, da), DvSub(dc, da))
	return Xpt(XhpStripTwosOwned(XhpOfDyadics(d[0], d[1], d[2]))), nil
}
