//go:build !race

package proof

import "math"

// These bit operations avoid the floating-point products of the race build's
// arithmetic steps. Positive finite encodings increase with the value;
// negative finite encodings decrease. Zeros, NaNs and infinities need their
// own cases before stepping, as in math.Nextafter.

// NextUp is math.Nextafter(x, +Inf), bit for bit, for every float64 x.
func NextUp(x float64) float64 {
	const sign = uint64(1) << 63
	const infinity = uint64(0x7ff0000000000000)
	b := math.Float64bits(x)
	magnitude := b &^ sign
	if magnitude >= infinity {
		if magnitude > infinity {
			return math.NaN()
		}
		if b == infinity {
			return x
		}
	}
	if magnitude == 0 {
		return 0x1p-1074
	}
	if b&sign == 0 {
		b++
	} else {
		b--
	}
	return math.Float64frombits(b)
}

// NextDown is math.Nextafter(x, -Inf), bit for bit, for every float64 x.
func NextDown(x float64) float64 {
	const sign = uint64(1) << 63
	const infinity = uint64(0x7ff0000000000000)
	b := math.Float64bits(x)
	magnitude := b &^ sign
	if magnitude >= infinity {
		if magnitude > infinity {
			return math.NaN()
		}
		if b == infinity|sign {
			return x
		}
	}
	if magnitude == 0 {
		return -0x1p-1074
	}
	if b&sign == 0 {
		b--
	} else {
		b++
	}
	return math.Float64frombits(b)
}

// Abs is math.Abs, bit for bit, for every float64 x.
func Abs(x float64) float64 { return math.Abs(x) }
