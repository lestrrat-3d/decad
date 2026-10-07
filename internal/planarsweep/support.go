package planarsweep

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// PlaneKey appends to b a key naming the plane through a with nonzero normal
// n: n's primitive integer direction, sign kept, and the exact offset d·a.
// Two keys are equal exactly when the normals point in the same direction
// and the offsets agree.
func PlaneKey(b []byte, n, a proofarith.DyV3) []byte {
	d := proofarith.DvPrimitive(n)
	for _, c := range d {
		b = c.AppendKey(b)
	}
	return proofarith.DvDot(d, a).AppendKey(b)
}

// SupportRuledOut proves that a guest vertex lies behind a plane or that no
// guest vertex can lie within its support band. Outward float enclosures may
// only reject: a NaN or inconclusive enclosure returns false. A true result
// holds only where ReadSupport reaches the same rejection.
func SupportRuledOut(boxesM []proofarith.FloatBox3, n, a proofarith.DyV3, band proofarith.Dyadic) bool {
	nBox := proofarith.DvFloatBox(n)
	cLo, cHi := proofarith.FloatBounds(proofarith.DvDot(n, a))
	// Every comparison below is false on a NaN end, so a NaN decides nothing.
	allAbove, minLo := true, math.Inf(1)
	for _, box := range boxesM {
		lo, hi := proofarith.DotSubEnclosure(nBox, box, cLo, cHi)
		if hi < 0 {
			return true
		}
		if !(lo > 0) {
			allAbove = false
			continue
		}
		minLo = math.Min(minLo, lo)
	}
	if !allAbove {
		return false
	}
	if band.Sign() <= 0 {
		return true
	}
	limit := proofarith.DyMul(proofarith.DyMul(band, band), proofarith.DvDot(n, n))
	return minLo > proofarith.DySqrtUp(limit)
}

// SupportRead contains the structural facts of one accepted support plane.
type SupportRead struct {
	Local   bool
	Heights []*big.Rat
	Contact []int
	Lifted  []int
}

// ReadSupport checks one plane against both vertex sets. A face-local plane
// requires a translating owner; the caller checks that its triangles belong
// to one flat face before using the result.
func ReadSupport(owner, guest []proofarith.DyV3, n, a proofarith.DyV3, band proofarith.Dyadic,
	movingOwner bool, poll func() error) (SupportRead, bool, error) {
	limit := proofarith.DyMul(proofarith.DyMul(band, band), proofarith.DvDot(n, n))
	local := false
	for _, v := range owner {
		if err := poll(); err != nil {
			return SupportRead{}, false, err
		}
		if proofarith.DvDot(n, proofarith.DvSub(v, a)).Sign() > 0 {
			local = true
			break
		}
	}
	if local && movingOwner {
		return SupportRead{}, false, nil
	}
	read := SupportRead{Local: local, Heights: make([]*big.Rat, len(guest))}
	for i, v := range guest {
		if err := poll(); err != nil {
			return SupportRead{}, false, err
		}
		height := proofarith.DvDot(n, proofarith.DvSub(v, a))
		switch height.Sign() {
		case -1:
			return SupportRead{}, false, nil
		case 0:
			read.Contact = append(read.Contact, i)
		default:
			if band.Sign() > 0 && proofarith.DyCmp(proofarith.DyMul(height, height), limit) <= 0 {
				read.Lifted = append(read.Lifted, i)
			}
		}
		read.Heights[i] = height.Rat()
	}
	if len(read.Contact) == 0 && len(read.Lifted) == 0 {
		return SupportRead{}, false, nil
	}
	return read, true, nil
}

// GridHorizon returns the largest fraction on the sweep's dyadic grid at
// which a monotone predicate holds.
func GridHorizon(resolution, duration *big.Rat,
	holds func(f *big.Rat) (bool, error)) (*big.Rat, bool, error) {
	ok, err := holds(big.NewRat(1, 1))
	if err != nil || ok {
		return big.NewRat(1, 1), ok, err
	}
	depth := uint(0)
	for depth < 52 && new(big.Rat).Mul(resolution, new(big.Rat).SetInt64(int64(1)<<depth)).Cmp(duration) < 0 {
		depth++
	}
	low, high := int64(0), int64(1)<<depth
	for high-low > 1 {
		mid := low + (high-low)/2
		holdsMid, err := holds(big.NewRat(mid, int64(1)<<depth))
		if err != nil {
			return nil, false, err
		}
		if holdsMid {
			low = mid
		} else {
			high = mid
		}
	}
	if low == 0 {
		return nil, false, nil
	}
	return big.NewRat(low, int64(1)<<depth), true, nil
}
