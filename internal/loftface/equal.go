package loftface

import "math/big"

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
