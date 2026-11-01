package proof

import (
	"math"
	"math/big"
	"math/bits"

	"github.com/lestrrat-3d/r3"
)

// TriangleNormalReplay charges each exact operation from its actual operand
// widths. Multiplication uses a schoolbook limb kernel with explicit storage.
func TriangleNormalReplay(a, b, c r3.Vec, reserve, spend func(uint64) error) (Xpt, error) {
	if reserve == nil || spend == nil {
		return Xpt{}, errReplayExactNormalNeedsSharedCounters
	}
	var vals [3][3]Dyadic
	for i, v := range [3]r3.Vec{a, b, c} {
		for j, f := range [3]float64{v.X, v.Y, v.Z} {
			if err := spend(1); err != nil {
				return Xpt{}, err
			}
			if math.IsNaN(f) || math.IsInf(f, 0) {
				return Xpt{}, errReplayExactNormalNeedsFiniteCoordinates
			}
			// DyOfFinite stores a binary64 significand inline, already stripped
			// of trailing zero bits, including for subnormal coordinates.
			vals[i][j] = DyOfFinite(f)
		}
	}
	var u, v [3]Dyadic
	for i := range 3 {
		if err := normalChargeDifference(vals[1][i], vals[0][i], reserve, spend); err != nil {
			return Xpt{}, err
		}
		u[i] = DySubScalar(vals[1][i], vals[0][i])
		if err := normalChargeDifference(vals[2][i], vals[0][i], reserve, spend); err != nil {
			return Xpt{}, err
		}
		v[i] = DySubScalar(vals[2][i], vals[0][i])
	}
	var normal [3]Dyadic
	for i := range 3 {
		j, k := (i+1)%3, (i+2)%3
		p, err := normalMulReplay(u[j], v[k], reserve, spend)
		if err != nil {
			return Xpt{}, err
		}
		q, err := normalMulReplay(u[k], v[j], reserve, spend)
		if err != nil {
			return Xpt{}, err
		}
		if err := normalChargeDifference(p, q, reserve, spend); err != nil {
			return Xpt{}, err
		}
		normal[i] = DySubScalar(p, q)
	}
	base, maxBits := 0, 1
	for _, d := range normal {
		if !d.IsZero() {
			base = min(base, d.exp)
		}
	}
	maxBits = max(maxBits, 1-base)
	for _, d := range normal {
		if !d.IsZero() {
			maxBits = max(maxBits, d.bitLen()+d.exp-base)
		}
	}
	words := uint64((maxBits + bits.UintSize - 1) / bits.UintSize)
	// Four shifted coordinates and their independently grown backing arrays,
	// plus in-place strip temporaries. Each allocation has four spare limbs.
	if err := normalReserveLimbs(reserve, 8*(words+4), 8); err != nil {
		return Xpt{}, err
	}
	if err := spend(16*words + 8); err != nil {
		return Xpt{}, err
	}
	return Xpt(XhpStripTwosOwned(XhpOfDyadics(normal[0], normal[1], normal[2]))), nil
}

func normalReserveLimbs(reserve func(uint64) error, limbs, objects uint64) error {
	if limbs > (^uint64(0)-63)/uint64(bits.UintSize/8) {
		return errReplayExactNormalStorageOverflow
	}
	blocks := (limbs*uint64(bits.UintSize/8) + 63) / 64
	if objects > ^uint64(0)-blocks {
		return errReplayExactNormalStorageOverflow
	}
	return reserve(blocks + objects)
}

func normalChargeDifference(a, b Dyadic, reserve, spend func(uint64) error) error {
	base, top := 0, 0
	if !a.IsZero() {
		base = a.exp
		top = a.exp + a.bitLen()
	}
	if !b.IsZero() {
		if a.IsZero() {
			base = b.exp
			top = b.exp + b.bitLen()
		} else {
			base = min(base, b.exp)
			top = max(top, b.exp+b.bitLen())
		}
	}
	width := max(1, top-base+1)
	words := uint64((width + bits.UintSize - 1) / bits.UintSize)
	// Two aligned inputs, output, normalized result and spare limbs for
	// math/big's add/subtract operations; no multiplication is used here.
	if err := normalReserveLimbs(reserve, 6*(words+4), 6); err != nil {
		return err
	}
	return spend(12*words + 6)
}

func normalMulReplay(a, b Dyadic, reserve, spend func(uint64) error) (Dyadic, error) {
	if a.IsZero() || b.IsZero() {
		if err := spend(1); err != nil {
			return Dyadic{}, err
		}
		return Dyadic{}, nil
	}
	aw := uint64((a.bitLen() + bits.UintSize - 1) / bits.UintSize)
	bw := uint64((b.bitLen() + bits.UintSize - 1) / bits.UintSize)
	// Two input views and one exact output array. The explicit kernel has no
	// recursive multiplication or hidden scratch allocation.
	if err := normalReserveLimbs(reserve, 2*(aw+bw)+8, 4); err != nil {
		return Dyadic{}, err
	}
	if err := spend(aw*bw + 3*(aw+bw) + 1); err != nil {
		return Dyadic{}, err
	}
	var ai, bi big.Int
	var ab, bb dyBuf
	ax, bx := a.view(&ai, &ab).Bits(), b.view(&bi, &bb).Bits()
	out := make([]big.Word, len(ax)+len(bx))
	for i, x := range ax {
		carry := uint(0)
		for j, y := range bx {
			hi, lo := bits.Mul(uint(x), uint(y))
			lo, c := bits.Add(lo, uint(out[i+j]), 0)
			hi, _ = bits.Add(hi, 0, c)
			lo, c = bits.Add(lo, carry, 0)
			hi, _ = bits.Add(hi, 0, c)
			out[i+j] = big.Word(lo)
			carry = hi
		}
		out[i+len(bx)] = big.Word(carry)
	}
	z := new(big.Int).SetBits(out)
	if (a.Sign() < 0) != (b.Sign() < 0) {
		z.Neg(z)
	}
	return fromBig(z, a.exp+b.exp), nil
}
