package motionbound

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
)

// ParamCompare orders two finite values of one Kind by the exact quantities
// they denote (MotionParam): −1, 0 or +1. An angle mixing whole turns and
// radians is compared through π's enclosure; ok is false when that
// enclosure cannot sign the difference, and every caller refuses then.
func ParamCompare(a, b units.Value) (int, bool) {
	pa, okA := ExactMotionParam(a)
	pb, okB := ExactMotionParam(b)
	if !okA || !okB {
		return 0, false
	}
	dTurn := new(big.Rat).Sub(pa.Turn, pb.Turn)
	dBase := new(big.Rat).Sub(pa.Base, pb.Base)
	switch {
	case dTurn.Sign() == 0:
		return dBase.Sign(), true
	case dBase.Sign() == 0:
		return dTurn.Sign(), true
	}
	twoPi := proofbound.TwoPiInterval()
	lo := new(big.Rat).Mul(dTurn, twoPi.Lo)
	hi := new(big.Rat).Mul(dTurn, twoPi.Hi)
	if lo.Cmp(hi) > 0 {
		lo, hi = hi, lo
	}
	lo.Add(lo, dBase)
	hi.Add(hi, dBase)
	switch {
	case lo.Sign() > 0:
		return 1, true
	case hi.Sign() < 0:
		return -1, true
	}
	return 0, false
}

// ParamLower and ParamUpper bound 2π·turn + base from below and above, π at
// its enclosure's ends.
func ParamLower(p MotionParam) *big.Rat {
	twoPi := proofbound.TwoPiInterval()
	f := twoPi.Lo
	if p.Turn.Sign() < 0 {
		f = twoPi.Hi
	}
	out := new(big.Rat).Mul(p.Turn, f)
	return out.Add(out, p.Base)
}

func ParamUpper(p MotionParam) *big.Rat {
	twoPi := proofbound.TwoPiInterval()
	f := twoPi.Hi
	if p.Turn.Sign() < 0 {
		f = twoPi.Lo
	}
	out := new(big.Rat).Mul(p.Turn, f)
	return out.Add(out, p.Base)
}
