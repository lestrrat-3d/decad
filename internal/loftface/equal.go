package loftface

import "math/big"

// AffineEqual reports whether two vertical affine records are equal.
func AffineEqual(a, b *VerticalAffine) bool {
	if a == nil || b == nil {
		return a == nil && b == nil
	}
	if !RatEqual(a.Det, b.Det) {
		return false
	}
	for axis := range a.Offset {
		if !RatEqual(a.Offset[axis], b.Offset[axis]) {
			return false
		}
	}
	for sourceAxis := range a.Column {
		for worldAxis := range a.Column[sourceAxis] {
			if !RatEqual(a.Column[sourceAxis][worldAxis],
				b.Column[sourceAxis][worldAxis]) {
				return false
			}
		}
	}
	return true
}

// LimitEqual reports whether two vertical limits are equal.
func LimitEqual(a, b VerticalLimit) bool {
	if a.Side != b.Side || a.Before != b.Before || a.After != b.After ||
		!IntervalEqual(a.Root, b.Root) {
		return false
	}
	for axis := range a.Box {
		if !IntervalEqual(a.Box[axis], b.Box[axis]) {
			return false
		}
	}
	return true
}

// PieceEqual reports whether two vertical pieces are equal.
func PieceEqual(a, b VerticalPiece) bool {
	if a.LoOpen != b.LoOpen || a.HiOpen != b.HiOpen ||
		!IntervalEqual(a.Source, b.Source) {
		return false
	}
	for axis := range a.Box {
		if !IntervalEqual(a.Box[axis], b.Box[axis]) {
			return false
		}
	}
	return true
}

// IntervalEqual reports whether two intervals have equal endpoints.
func IntervalEqual(a, b Interval) bool {
	return RatEqual(a.Lo, b.Lo) &&
		RatEqual(a.Hi, b.Hi)
}

// RatEqual reports whether a and b are both nil or equal.
func RatEqual(a, b *big.Rat) bool {
	if a == nil || b == nil {
		return a == nil && b == nil
	}
	return a.Cmp(b) == 0
}
