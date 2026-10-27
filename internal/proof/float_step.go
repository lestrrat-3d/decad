package proof

import "math"

// NextUp is math.Nextafter(x, +Inf), bit for bit, for every float64 x.
//
// For |x| in [2^-969, MaxFloat64) the neighbour is formed by float arithmetic
// rather than by bit manipulation: with x = m·2^E (m in [1, 2)) the product
// x·2^-53 is exact and normal, and x + x·2^-53 lies between half an ulp and a
// whole ulp above x, so round-to-nearest lands on x + ulp — except at m = 1,
// where the sum is the exact tie and rounds back to the even x; x + x·2^-52 is
// then x + ulp exactly. Below zero the step toward zero is x − x·2^-53, which
// is never a tie. Everything else — zero, subnormals, the range ends,
// infinities and NaN — takes math.Nextafter itself. The float64() conversions
// forbid fusing the product into the sum.
func NextUp(x float64) float64 {
	if x >= 0x1p-969 && x < math.MaxFloat64 {
		y := x + float64(x*0x1p-53)
		if y == x {
			y = x + float64(x*0x1p-52)
		}
		return y
	}
	if x <= -0x1p-969 && x >= -math.MaxFloat64 {
		return x - float64(x*0x1p-53)
	}
	if x == 0 {
		return 0x1p-1074
	}
	return math.Nextafter(x, math.Inf(1))
}

// NextDown is math.Nextafter(x, -Inf), bit for bit; NextUp's mirror.
func NextDown(x float64) float64 {
	if x <= -0x1p-969 && x > -math.MaxFloat64 {
		y := x + float64(x*0x1p-53)
		if y == x {
			y = x + float64(x*0x1p-52)
		}
		return y
	}
	if x >= 0x1p-969 && x <= math.MaxFloat64 {
		return x - float64(x*0x1p-53)
	}
	if x == 0 {
		return -0x1p-1074
	}
	return math.Nextafter(x, math.Inf(-1))
}

// Abs is math.Abs, bit for bit, by comparison instead of a bit cast.
// Under -race, the build CI tests with, math.Abs's bit cast is checked and
// race-instrumented on every call and runs several times slower than Abs.
// The plain build favours math.Abs, so measure both builds before switching.
func Abs(x float64) float64 {
	if x < 0 {
		return -x
	}
	if x > 0 {
		return x
	}
	if x == 0 {
		return 0
	}
	return math.Abs(x)
}
