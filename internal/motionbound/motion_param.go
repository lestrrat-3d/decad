package motionbound

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
)

// MotionParam is one motion parameter's exact denotation. An angle denotes
// θ = 2π·turn + base radians: a degree-stated angle is an exact rational turn
// (units.Degree's own factor is a rounded π/180, so the degree count, not the
// factor, is what the caller stated), and any other angle unit is base =
// magnitude × factor radians, read exactly. A length denotes base millimetres
// and leaves turn zero, and a Between's dimensionless fraction denotes base
// and leaves turn zero.
type MotionParam struct {
	Turn *big.Rat
	Base *big.Rat
}

// ExactMotionParam reads v's exact denotation. It fails only on a non-finite
// magnitude or factor.
func ExactMotionParam(v units.Value) (MotionParam, bool) {
	mag := proofarith.FloatRat(v.Mag())
	if mag == nil {
		return MotionParam{}, false
	}
	if v.Unit() == units.Degree {
		return MotionParam{Turn: new(big.Rat).Quo(mag, big.NewRat(360, 1)), Base: new(big.Rat)}, true
	}
	factor := proofarith.FloatRat(v.Unit().Factor())
	if factor == nil {
		return MotionParam{}, false
	}
	return MotionParam{Turn: new(big.Rat), Base: new(big.Rat).Mul(mag, factor)}, true
}

// lerp is p + f·(q − p), exactly.
func (p MotionParam) Lerp(q MotionParam, f *big.Rat) MotionParam {
	at := func(a, b *big.Rat) *big.Rat {
		d := new(big.Rat).Sub(b, a)
		return d.Add(a, d.Mul(d, f))
	}
	return MotionParam{Turn: at(p.Turn, q.Turn), Base: at(p.Base, q.Base)}
}

// spanUpper is a proven upper bound on |θ(q) − θ(p)| in the base unit:
// 2π·|Δturn| + |Δbase|, with π taken at its upper enclosure. It is exact for
// a length, whose turn is always zero.
func (p MotionParam) SpanUpper(q MotionParam) *big.Rat {
	dTurn := new(big.Rat).Sub(q.Turn, p.Turn)
	dTurn.Abs(dTurn)
	dBase := new(big.Rat).Sub(q.Base, p.Base)
	dBase.Abs(dBase)
	out := new(big.Rat).Mul(dTurn, proofbound.TwoPiInterval().Hi)
	return out.Add(out, dBase)
}

// ExactTurnSinCos encloses sin(2πt) and cos(2πt) for an exact rational turn.
// A whole number of quarter turns answers exactly — that is what keeps an
// identity pose (the parameter 0) and a right-angle pose free of any
// enclosure width — and every other turn takes proofbound.TurnSinCosInterval.
func ExactTurnSinCos(t *big.Rat) (proofbound.RatInterval, proofbound.RatInterval) {
	q := new(big.Rat).Mul(t, big.NewRat(4, 1))
	if q.IsInt() {
		k := new(big.Int).Mod(q.Num(), big.NewInt(4)).Int64()
		sinCos := [4][2]int64{{0, 1}, {1, 0}, {0, -1}, {-1, 0}}[k]
		return proofbound.PointInterval(big.NewRat(sinCos[0], 1)), proofbound.PointInterval(big.NewRat(sinCos[1], 1))
	}
	return proofbound.TurnSinCosInterval(t)
}

// RadianSinCos encloses sin(r) and cos(r) for an exact rational angle r in
// radians. r is the turn r/2π, which lies in [r/(2·πhi), r/(2·πlo)] for a
// non-negative r (reversed for a negative one); the pair is evaluated at the
// lower turn and widened by the turn interval's own angular width, since sine
// and cosine are 1-Lipschitz in the angle: |sin(2πt) − sin(2πt_lo)| ≤
// 2π·(t_hi − t_lo) ≤ 2·πhi·(t_hi − t_lo).
//
// A nonzero angle's pair is read through the process-wide memo
// (sin_cos_memo.go); the caller owns the endpoints it gets either way.
func RadianSinCos(r *big.Rat) (proofbound.RatInterval, proofbound.RatInterval) {
	if r.Sign() == 0 || !MemosOn() {
		return radianSinCos(r)
	}
	key := string(AppendRatKey(nil, r))
	if entry, ok := sinCosMemo.load(key); ok {
		return entry.sin, entry.cos
	}
	sin, cos := radianSinCos(r)
	sinCosMemo.store(key, sinCosEntry{sin: sin, cos: cos})
	return sin, cos
}

// radianSinCos is RadianSinCos computed afresh.
func radianSinCos(r *big.Rat) (proofbound.RatInterval, proofbound.RatInterval) {
	if r.Sign() == 0 {
		return proofbound.PointInterval(new(big.Rat)), proofbound.PointInterval(big.NewRat(1, 1))
	}
	twoPi := proofbound.TwoPiInterval()
	tA := new(big.Rat).Quo(r, twoPi.Hi)
	tB := new(big.Rat).Quo(r, twoPi.Lo)
	tLo, tHi := tA, tB
	if tLo.Cmp(tHi) > 0 {
		tLo, tHi = tHi, tLo
	}
	sin, cos := proofbound.TurnSinCosInterval(tLo)
	width := new(big.Rat).Sub(tHi, tLo)
	width.Mul(width, twoPi.Hi)
	widen := func(iv proofbound.RatInterval) proofbound.RatInterval {
		return proofbound.IntervalOwned(new(big.Rat).Sub(iv.Lo, width), new(big.Rat).Add(iv.Hi, width))
	}
	return widen(sin), widen(cos)
}

// ParamSinCos encloses sin θ and cos θ for θ = 2π·turn + base, by the angle
// sum formulas over the two parts' own enclosures. A part that is exactly zero
// contributes the point pair (0, 1), so a pure-degree or pure-radian angle
// passes its own enclosure through unwidened.
func ParamSinCos(p MotionParam) (proofbound.RatInterval, proofbound.RatInterval) {
	sT, cT := ExactTurnSinCos(p.Turn)
	if p.Base.Sign() == 0 {
		return sT, cT
	}
	sR, cR := RadianSinCos(p.Base)
	if p.Turn.Sign() == 0 {
		return sR, cR
	}
	sin := proofbound.IntervalAdd(proofbound.IntervalMul(sT, cR), proofbound.IntervalMul(cT, sR))
	cos := proofbound.IntervalSub(proofbound.IntervalMul(cT, cR), proofbound.IntervalMul(sT, sR))
	return sin, cos
}
