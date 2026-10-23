package loftface

import "math/big"

// Mapped arc band outcomes.
const (
	ArcMiss    = 0
	ArcInside  = -1
	ArcOutside = 1
)

// RatMax returns the larger of a and b.
func RatMax(a, b *big.Rat) *big.Rat {
	if a.Cmp(b) > 0 {
		return a
	}
	return b
}

// ArcCapRanges returns the X, Y and Z ranges of the cap map over the source
// corners.
func ArcCapRanges(mapped Map, corners [4][2]*big.Rat) [3]Interval {
	return cornerAxisRanges(mapped, corners)
}

// cornerAxisRanges evaluates the map once at each corner and returns the
// X, Y and Z ranges of the four images. Each interval holds fresh copies.
func cornerAxisRanges(m Map, corners [4][2]*big.Rat) [3]Interval {
	var images [4]RatVec
	for i, corner := range corners {
		images[i] = m.At(corner[0], corner[1])
	}
	var out [3]Interval
	for axis := range out {
		for i := range images {
			value := new(big.Rat).Set(images[i][axis])
			if i == 0 {
				out[axis] = Interval{value, value}
				continue
			}
			if value.Cmp(out[axis].Lo) < 0 {
				out[axis].Lo = value
			}
			if value.Cmp(out[axis].Hi) > 0 {
				out[axis].Hi = value
			}
		}
	}
	return out
}

// RadialCap is the exact X and the source U/V of one radial cap.
type RadialCap struct {
	X    *big.Rat
	U, V [2]*big.Rat
}
