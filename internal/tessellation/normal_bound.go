package tessellation

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func IvVec3Sub(a, b survey2d.IvVec3) survey2d.IvVec3 {
	return survey2d.IvVec3{proofbound.IntervalSub(a[0], b[0]), proofbound.IntervalSub(a[1], b[1]), proofbound.IntervalSub(a[2], b[2])}
}

func IvVec3Cross(a, b survey2d.IvVec3) survey2d.IvVec3 {
	return survey2d.IvVec3{
		proofbound.IntervalSub(proofbound.IntervalMul(a[1], b[2]), proofbound.IntervalMul(a[2], b[1])),
		proofbound.IntervalSub(proofbound.IntervalMul(a[2], b[0]), proofbound.IntervalMul(a[0], b[2])),
		proofbound.IntervalSub(proofbound.IntervalMul(a[0], b[1]), proofbound.IntervalMul(a[1], b[0])),
	}
}

// RadSinCosSpan is survey2d.RadSinCosInterval over a whole radian INTERVAL rather than
// one exact radian value — what a caller holds when the angle itself is only
// enclosed, as an arc's own a0 + t·sweep is (both terms come from
// proofbound.Atan2Interval).
//
// It evaluates the point enclosure at the span's lower end and widens both
// readings by the span's own width. That is sound because sine and cosine are
// 1-Lipschitz, so no monotonicity over the span need be argued — the same
// argument survey2d.RadSinCosInterval makes for its own grid gap, one level up.
func RadSinCosSpan(x proofbound.RatInterval) (proofbound.RatInterval, proofbound.RatInterval, bool) {
	width := new(big.Rat).Sub(x.Hi, x.Lo)
	if width.Sign() < 0 {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, false
	}
	sin, cos, ok := survey2d.RadSinCosInterval(x.Lo)
	if !ok {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, false
	}
	return survey2d.IntervalWiden(sin, width), survey2d.IntervalWiden(cos, width), true
}
